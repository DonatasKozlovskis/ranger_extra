#!/usr/bin/env python


#######################
# Import required Python code.
import rospy

import actionlib
import csv
import os
import sys
import pdb

# Transformations


# ROS messages
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import *
from visualization_msgs.msg import Marker
from std_msgs.msg import String
# ROS server messages
from std_srvs.srv import Empty


###############################################
#
#   Class for the Mapper.py ROS node
class Navigator():
    #
    # Constructor
    #
    #   path_map: path where the map file will be saved
    #   path_waypoint: path where the waypoint file will be saved

    def __init__(self, full_path_map, full_path_waypoint):
        '''
        constructor
        '''
        # take params from init
        self.full_path_waypoint = full_path_map
        self.full_path_waypoint = full_path_waypoint
        
        # Get the private namespace parameters from launch file.
        self.fixed_frame = rospy.get_param('~fixed_frame', 'map')
        
        # rate 
        self.rate = rospy.Rate(10.0) #10 Hz
                  
        # waypoint file format
        self.wp_fieldnames = ('num_id', 'wp_name', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw')                            
        self.waypoints = self.loadWaypoints();

        # Publisher of visualization markers for waypoints
        self.marker_pub = rospy.Publisher("visualization_marker", Marker, queue_size=40)
        # service to clear the costmaps
        self.clear_costmaps = rospy.ServiceProxy("/move_base/clear_costmaps", Empty)
        # Subscribe for keyboard        
        rospy.Subscriber('keypress_talker',      String,     self.key_callback)


        #create action client
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
        self.visualizeMarkers()
        
        while not rospy.is_shutdown():
            
#            wp = self.getWaypointByName("Frame_3")
#            rospy.loginfo(  "Found Waypoint %s" % wp)

            self.rate.sleep()
            
        return 0

    
    #==========================================================================
    # read waypoint files
    #
    # each line has one waypoint in format:
    # ('num_id', 'name', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw')                            
    #
    #   x,y,z:     x,y,z of waypoint location in map frame
    #   qx,qy,qz,qw:  (quaternion components) in map frame
    #
    def loadWaypoints(self):

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
    def visualizeMarkers(self):
        index = 1
        for waypoint in self.waypoints:
            # publish arrow and text markers
            self.marker_pub.publish( self.make_marker_text(index, waypoint) )
            self.marker_pub.publish( self.make_marker_arrow(index, waypoint) )
            index +=1
            # sleep between publishing, otherwise messages will not be published
            self.rate.sleep()
            
    #==========================================================================
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
    def getWaypointByName(self, name):
        try:
            waypoint = (wp for wp in self.waypoints if wp["wp_name"] == name).next()
        except StopIteration as ex:
            rospy.logwarn("getWaypointByName failed %s" % str(ex))
            waypoint = {}
        return waypoint
        
    #==========================================================================
    def getWaypointByIndex(self, index):
        try:
            waypoint = self.waypoints[index]
        except:
            rospy.logwarn("getWaypointByIndex failed")
            waypoint = {}
        return waypoint

    #==========================================================================  
    def createGoal(self, waypoint):
        
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

    def goTo(self, waypoint):
        
        goal = self.createGoal(waypoint)

        #set goal, start moving
        self.move_base.send_goal(goal)

        #allow robot up to 60 seconds to complete task
        success = self.move_base.wait_for_result(rospy.Duration(60)) 


        if not success:
            self.move_base.cancel_goal()
            rospy.loginfo("Move failed, time out 60 s")
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
    def key_callback(self,msg):
        '''
        function to handle keyboard buttons
        '''
        
        key = str(msg.data)
        #rospy.loginfo("Heard keypress %s", key)
        
        if (key.isdigit()):
            index = int(key)
            if (index >= 0 and index < len(self.waypoints)):
                waypoint = self.getWaypointByIndex(index)
                self.goTo(waypoint)
            else:
                rospy.logwarn("key index out of bounds");
                
        else:
            rospy.logwarn("not a number keypress");
        
        

#==============================================================================
# main function
if __name__=="__main__":
    
    sts = 0
    rospy.loginfo("Running Navigator")
    try:
        # ROS initialization
        rospy.init_node("navigator")
        if sts ==0:
            
            # params to pass to the class
            full_path_map = ""
            full_path_waypoint =  ""
      

             # param checks
            if full_path_map == "":    
                full_path_map = rospy.get_param("/navigator/file_map","")
            if full_path_waypoint == "":    
                full_path_waypoint = rospy.get_param("/navigator/file_waypoint","") 
    
            if full_path_map == "":
                rospy.loginfo(  "Error: No map file specified")
                sts= 1
            else:
                # check if file exists
                if (os.path.isfile(full_path_map)):
                    rospy.loginfo(  "Map file: '%s'" % full_path_map)
                else:
                    rospy.loginfo(  "Error: Map file '%s' not found"  % full_path_map )
                    sts= 1
                    
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
            navigator = Navigator(full_path_map,full_path_waypoint)
            sts = navigator.run()
        
    except Exception as ex:
        rospy.loginfo("Navigator Crashed with exception: %s" % str(ex))
        sts = -1
    finally:
        rospy.loginfo("Navigator Finished")
        sys.exit(sts)





