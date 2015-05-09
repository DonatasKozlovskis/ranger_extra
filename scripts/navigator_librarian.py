#!/usr/bin/env python


#######################
# Import required Python code.
import rospy

import actionlib
import csv
import os
import sys
import pdb
import numpy

# Transformations


# ROS messages
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import *
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_srvs.srv import Empty
from ranger_librarian.msg import NavigatorAction


###############################################
#
#   Class for the Mapper.py ROS node
class Navigator():
    #
    # Constructor
    #
    #   path_map: path where the map file will be saved
    #   path_waypoint: path where the waypoint file will be saved
    NODE_FREQUENCY = 5 #Hz

    def __init__(self, full_path_waypoint):
        '''
        constructor
        '''
        # take params from init
        self.full_path_waypoint = full_path_waypoint
        
        # Get the private namespace parameters from launch file.
        self.fixed_frame = rospy.get_param('~fixed_frame', 'map')
        
        # rate 
        self.rate = rospy.Rate(self.NODE_FREQUENCY) #NODE_FREQUENCY Hz
        
        # goal tries 
        goal_time = 180 # allow 180/60=3 minutes to reach goal
        self.goal_tries_max = goal_time*self.NODE_FREQUENCY;
        # action
        self.action_current = None;   #initial action
        self.action_before = None;   #initial action
        self.goal_wp_index =  0;       # gaols are defined in 2--nrows(waypoints)
        self.goal_waypoint = None;
                  
        # waypoint file format
        self.wp_fieldnames = ('num_id', 'wp_name', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw')                            
        self.waypoints = self.load_waypoints();

        # Publisher of visualization markers for waypoints
        self._pub_marker = rospy.Publisher("visualization_marker", Marker, queue_size=40)
        self._pub_wp_names = rospy.Publisher("frame_names", String, queue_size=40)
        self._pub_cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        
        # service to clear the costmaps
        self.clear_costmaps = rospy.ServiceProxy("/move_base/clear_costmaps", Empty)
        # Subscribe for navigator actions        
        rospy.Subscriber('navigator_action', NavigatorAction, self.nav_action_callback)

        #create move base actionlib client
        self.move_base = actionlib.SimpleActionClient("move_base" ,MoveBaseAction)

        # wait for move_base server to be ready
        while not self.move_base.wait_for_server(rospy.Duration(5.0)) and not rospy.is_shutdown():
            rospy.loginfo(  "Waiting for Move Base Server")
        
            
    #==========================================================================
    def run(self):
        '''
        main run loop
        '''
        rospy.loginfo("Navigator Ready")
        
        # Publish to RVIZ all existing markers        
        self.visualize_markers()
        
        goal_status = None;
        goal_tries = 0; # number of times robot tried to reach goal
        time_start = rospy.Time.now()       #current time
        time_now = rospy.Time.now()         #current time
        
#        self.go_straight(0.5);
#        self.look_around();
        
                
#publish loaded waypoint names
#            if (self._pub_wp_names.get_num_connections() > 0):
#                self._pub_wp_names.publish( String(self.get_waypoint_names()) )
        
        while not rospy.is_shutdown():
            
            if (self.action_current==NavigatorAction.STOP):
                # no need to loop over action stop
                if (1):
                    self.action_before = self.action_current
                    rospy.loginfo("action STOP")
                    if (self.goal_waypoint!=None):
                        self.move_base.wait_for_result(rospy.Duration(0.2))
                        self.move_base.cancel_all_goals();
                        self.clear_costmaps();
                        
                        self.goal_waypoint = None
                        self.goal_wp_index -= 1;
            # end if                         
            
            if (self.action_current==NavigatorAction.MOVE):
                rospy.loginfo("action MOVE");
                
                next_goal = self.goal_waypoint==None or goal_status == True or goal_tries>self.goal_tries_max
                if (next_goal):
                    # clear all goals if exist
                    self.move_base.cancel_all_goals();
                    self.clear_costmaps()
                    goal_tries = 0;
                     #create next goal
                    self.goal_waypoint = self.get_next_waypoint();
                    goal = self.create_goal(self.goal_waypoint)
                    #set goal, start moving
                    self.move_base.send_goal(goal)
                goal_tries += 1;
                goal_status = self.move_base.wait_for_result(rospy.Duration(0.5))
            # end if                                 

                
            if (self.action_current==NavigatorAction.FINISH):
                rospy.loginfo(  "action FINISH")
                goal_finish = self.action_before != self.action_current or goal_tries>self.goal_tries_max
                
                self.action_before = self.action_current
                if (goal_finish):
                    self.move_base.wait_for_result(rospy.Duration(0.1))
                    self.move_base.cancel_all_goals();
                    self.clear_costmaps();
                    goal_tries = 0;
                    # create librarian goal
                    # self.goal_waypoint = self.get_waypoint_by_name("Librarian")
                    # equivalent to
                    self.goal_waypoint = self.get_waypoint_by_index(0);
                    goal = self.create_goal(self.goal_waypoint)
                    #set goal, start moving
                    self.move_base.send_goal(goal)
                goal_tries += 1;
                goal_status = self.move_base.wait_for_result(rospy.Duration(10)) 
            # end if 
                
            self.rate.sleep();
            time_now = rospy.Time.now()
        # end while
            
        # cancel all goals before shutdown
        self.move_base.cancel_all_goals();
        # clear costmaps
        self.clear_costmaps();
        return 0
    #==========================================================================
    # get next waypoint by goal index
    # goal wp index lie in range [2,len(waypoints)] - total len(waypoints)-1 wp
    # thus temp index is in range [0-(len(waypoints)-2)]
    # 
    def get_next_waypoint(self):
        number_of_goal_waypoints = len(self.waypoints) -1;
        
        temp_index = self.goal_wp_index-1;
        # check just in case...
        if (temp_index<0):
            temp_index = 0;
        #increase index
        temp_index +=1;
        temp_index = temp_index % number_of_goal_waypoints;
        self.goal_wp_index = temp_index + 1;
        waypoint = self.get_waypoint_by_index(self.goal_wp_index)
        
        return waypoint;
        
    #==========================================================================
    # read waypoint files
    #
    # each line has one waypoint in format:
    # ('num_id', 'name', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw')                            
    #
    #   x,y,z:     x,y,z of waypoint location in map frame
    #   qx,qy,qz,qw:  (quaternion components) in map frame
    #
    def load_waypoints(self):

        waypoints = []
        
        with open(self.full_path_waypoint, 'r') as wpfile:
            reader = csv.DictReader(wpfile, delimiter=';')
            for row in reader:
                # convert to float
                row["x"] = float(row["x"])
                row["y"] = float(row["y"])
                row["z"] = float(row["z"])
                
                row["qx"] = float(row["qx"])
                row["qy"] = float(row["qy"])
                row["qz"] = float(row["qz"])
                row["qw"] = float(row["qw"])
                
                waypoints.append(row)
                
        rospy.loginfo(  "Loaded %d waypoints" % len(waypoints))
        return waypoints
    #==========================================================================
    # publish waypoints using arrows and names
    def visualize_markers(self):
        index = 1
        for waypoint in self.waypoints:
            # publish arrow and text markers
            self._pub_marker.publish( self.make_marker_text(index, waypoint) )
            self._pub_marker.publish( self.make_marker_arrow(index, waypoint) )
            index +=1
            # sleep between publishing, otherwise messages will not be published
            self.rate.sleep()
            
    #==========================================================================
    # create general marker
    def make_marker(self, marker_id, waypoint): 
        # method to create base RVIZ marker
        marker = Marker()
        marker.header.frame_id = "/" + self.fixed_frame
        marker.header.stamp = rospy.Time.now()
        marker.ns = "navigator"
        marker.id = marker_id
        
        marker.action = marker.ADD
        
        
        marker.pose.position.x = waypoint["x"]
        marker.pose.position.y = waypoint["y"]
        marker.pose.position.z = waypoint["z"]
        
        marker.lifetime = rospy.Duration()
        return marker
        
    #==========================================================================
    # create arrow marker
    def make_marker_arrow(self, marker_id, waypoint): 
        # method to create RVIZ marker
        marker = self.make_marker(marker_id, waypoint)

        marker.type= marker.ARROW
        
        marker.pose.position.z += 0.3

        marker.pose.orientation.x = 0.70711;
        marker.pose.orientation.y = 0;
        marker.pose.orientation.z = -0.70711;
        marker.pose.orientation.w = 0;
        
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        
        marker.scale.x = 0.3
        marker.scale.y = 0.02
        marker.scale.z = 0.02
        return marker

    #==========================================================================
    # create text marker
    def make_marker_text(self, marker_id, waypoint): 
        # method to create RVIZ marker
        marker = self.make_marker(marker_id, waypoint)
        marker.id = marker_id + 1000;
        
        marker.type= marker.TEXT_VIEW_FACING
        marker.text = waypoint["wp_name"]
        
        marker.pose.position.z += 0.35
        marker.pose.orientation.w = 1.0;
        marker.scale.z = 0.07
        
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
                
        return marker

    #==========================================================================
    def get_waypoint_names(self):
        wp_names = ""
        for waypoint in self.waypoints:
            wp_names += '\n' + waypoint.get('wp_name') 

        return wp_names

    #==========================================================================
    def get_waypoint_by_name(self, name):
        try:
            waypoint = (wp for wp in self.waypoints if wp["wp_name"] == name).next()
        except StopIteration as ex:
            rospy.logwarn("get_waypoint_by_name failed %s" % str(ex))
            waypoint = {}
        return waypoint
        
    #==========================================================================
    def get_waypoint_by_index(self, index):
        # list index in range [0-(len-1)]
        if (index >= 0 and index < len(self.waypoints)):
            waypoint = self.waypoints[index]
        else:
            rospy.logwarn("waypoint index out of bounds!")
            rospy.logwarn("%s" % str(index))
        waypoint = {}
        try:
            waypoint = self.waypoints[index]
        except:
            rospy.logwarn("get_waypoint_by_index failed")
            waypoint = {}
        return waypoint

    #==========================================================================  
    def create_goal(self, waypoint):
        
        goal = MoveBaseGoal()
        
        goal.target_pose.header.frame_id = self.fixed_frame
        goal.target_pose.header.stamp = rospy.Time.now()
        # create goal pose position
        goal.target_pose.pose.position.x = waypoint["x"]
        goal.target_pose.pose.position.y = waypoint["y"]
        goal.target_pose.pose.position.z = waypoint["z"]
        # create goal pose orientation
        goal.target_pose.pose.orientation.x  = waypoint["qx"]
        goal.target_pose.pose.orientation.y  = waypoint["qy"]
        goal.target_pose.pose.orientation.z  = waypoint["qz"]
        goal.target_pose.pose.orientation.w  = waypoint["qw"]
        
        return goal

    #==========================================================================

    def go_to(self, waypoint):
        
        goal = self.create_goal(waypoint)
        #set goal, start moving
        self.move_base.send_goal(goal)
        #allow robot up to 30 seconds to complete task
        success = self.move_base.wait_for_result(rospy.Duration(30)) 


        if not success:
            self.move_base.cancel_goal()
            rospy.loginfo("Move failed, time out 30 s")
        else:
            # We made it!
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("Move to goal succeeded")       
            else:
                rospy.loginfo("Move to goal failed")

        # clear costmaps
        self.clear_costmaps()
                   
    #==========================================================================
    def nav_action_callback(self,msg):
        '''
        function to handle navigator action callbacks
        '''
        action = msg.action;
        if (self.action_current<>action):
            # update actions
            self.action_current = action;
    #==========================================================================
    # function to turn robot around        
    def look_around(self):
        lookaround_time = 20
        tick = 0.2
        clock = 0.0
        # cancel the current waypoint goal
        self.move_base.cancel_all_goals()

        # clear costmaps
        self.clear_costmaps()
        
        # rotate 360 degrees with a small forward motion
        cmd_vel = Twist()
        cmd_vel.angular.z = 0.32
        cmd_vel.linear.x = 0.02
        
        while clock < lookaround_time:
            self._pub_cmd_vel.publish(cmd_vel)
            rospy.sleep(tick)
            clock += tick

        self._pub_cmd_vel.publish(Twist())

    #==========================================================================
    # function to move robot straith given
    # distance in meters (negative -backwards)  
    def go_straight(self, distance):
        # cancel the current waypoint goal
        self.move_base.cancel_all_goals()
        
        # knowing that max speed is 0.16 m/s
        moving_time = numpy.abs(distance)/0.16;
        tick = 0.2
        clock = 0.0
        cmd_vel = Twist()
        cmd_vel.angular.z = 0.0
        cmd_vel.linear.x = 0.16*numpy.sign(distance)
        
        while clock < moving_time:
            self._pub_cmd_vel.publish(cmd_vel)
            rospy.sleep(tick)
            clock += tick

#==============================================================================
# main function
if __name__=="__main__":
    
    sts = 0
    rospy.loginfo("Running Navigator")
    try:
        # ROS initialization
        rospy.init_node("navigator_librarian")
        if sts ==0:
            
            # params to pass to the class
            full_path_map = ""
            full_path_waypoint =  ""
      

             # param checks
            if full_path_waypoint == "":    
                full_path_waypoint = rospy.get_param("/navigator/file_waypoint","") 
                        
            if full_path_waypoint == "":
                rospy.loginfo(  "Error: No waypoint file specified")
                sts= 1
            else:
                # check if file exists
                if (os.path.isfile(full_path_waypoint)):
                    rospy.loginfo(  "Waypoint file: '%s'" % full_path_waypoint)
                else:
                    rospy.loginfo(  "Error: Waypoint file '%s' not found" % full_path_waypoint)
                    sts= 1


        if sts ==0:
            # run the navigator node
            navigator = Navigator(full_path_waypoint)
            sts = navigator.run()
        
    except Exception as ex:
        rospy.loginfo("Navigator Crashed with exception: %s" % str(ex))
        sts = -1
    finally:
        rospy.loginfo("Navigator Finished")
        sys.exit(sts)





