#!/usr/bin/env python


#######################
#
#   Manages the creation of the waypoint file
#   Handles saving of the grid 2d MAP and rtabmap.db files
#   Waypoints locations are saved based on the current pose defined in the /map to /base_footprint transform at the time of recording the WP.
#   The pose is forced to be in the 2d map plane
#
#   Uses keyboard button clicks to trigger the waypoint creation and map saving
#
#######################
# Import required Python code.
import rospy

import tf

import os
import sys
import subprocess

# Transformations
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# ROS messages
from std_srvs.srv import Empty
from std_msgs.msg import Header

from visualization_msgs.msg import Marker

###############################################
#
#   Class for the Mapper.py ROS node
class Mapper():
    #
    # Constructor
    #
    #   path_map: path where the map file will be saved
    #   path_waypoint: path where the waypoint file will be saved

    def __init__(self):
        '''
        constructor
        '''
        # Get the private namespace parameters from launch file.
        # folder locations
        self.path_map = rospy.get_param('~path_map', 'maps')
        self.path_waypoint =  rospy.get_param('~path_waypoint', 'maps')
        
        self.file_map = rospy.get_param('~file_map', 'map')
        self.file_waypoint = rospy.get_param('~file_waypoint', 'waypoints.txt')
        
        self.full_path_waypoint = os.path.join(self.path_waypoint, self.file_waypoint)
        
        # ensure given directories exist, if not create
        if not os.path.isdir(self.path_map):
            os.makedirs(self.path_map)
            
        if not os.path.isdir(self.path_waypoint):
            os.makedirs(self.path_waypoint) 
        # delete old waypoint file if exist
        try:
            os.remove(self.full_path_waypoint)
        except OSError:
            pass            
            
                            
        # Subscriber for keyboards
        rospy.Subscriber("mapper_control", Header, self.control_callback)
        
        # Publisher for visualization markers for waypoints
        self.marker_pub = rospy.Publisher("visualization_marker", Marker, queue_size=5)

        # need the transform from  /map to /base_footprint
        self.listener = tf.TransformListener()
        
        self.rate = rospy.Rate(10.0) #10 Hz
        
        self.tf = None          # not seen a transform yet
        self.waypoint_num = 0   # waypoint enumerator
        
        self.saved_frames = {};
                
        # give tf a chance to queue up some transforms
        rospy.sleep(3)
    
    #==========================================================================
    def run(self):
        '''
        main run loop
        '''
        rospy.loginfo("Map path: %s",       self.path_map);
        rospy.loginfo("Waypoint path: %s",  self.path_waypoint);
        rospy.loginfo("Mapper Ready")

        while not rospy.is_shutdown():
            transform_ok = False
            try:
                # check for transform
                (trans, rot) = self.listener.lookupTransform('odom', 'base_link', rospy.Time(0))

                # clean up pose, force to be on and parallel to map plane
                trans = (trans[0], trans[1], 0) # z=0

                # roll and pitch = 0
                (r, p, y) = euler_from_quaternion(rot)
                qt = quaternion_from_euler(0, 0, y)
                rot=(qt[0], qt[1], qt[2], qt[3])
                
                transform_ok = True

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass # ignore these exceptions
                
            if transform_ok: # good transform so save it in case we save a waypoint
                self.tf = (trans, rot)
        
            self.rate.sleep()
        
        return 0

    
    #==========================================================================
    def control_callback(self, msg):
        '''
        function to handle keyboar buttons
        '''
#        uint32 seq
#        string frame_id
        command_key = msg.seq
        frame_name    = msg.frame_id
        
#        rospy.loginfo("Heard keypress %d", command_key)
                    
        if command_key == 1:            
            self.add_waypoint(frame_name);
            
        if command_key == 2:   
            rospy.loginfo("Deleting waypoint: %s not implemented", frame_name);
            
        if command_key == 3:
            rospy.loginfo("Saving map and switching to localisation mode");
            self.save_map();

    #==========================================================================        
    def add_waypoint(self, frame_name):
        '''
        function to add waypoint to file
        '''
        
        if self.tf != None: # do we have a valid transform to use    
            #save in the waypoint file and create a visualization marker
         
            rospy.loginfo("Saving waypoint %s: %s, %s" %( str(frame_name), str(self.tf[0]), str(self.tf[1])  ) )

            # save waypoint pose in file
            with open(self.full_path_waypoint,'a+') as wpfh:
                wpfh.write("%s: %s\n" % (frame_name, str(self.tf)))

            rospy.loginfo("Waypoint %s saved." % frame_name)
            
            self.marker_pub.publish( self.make_marker_arrow() )
            self.marker_pub.publish( self.make_marker_text(frame_name) )   
            
            # increment waypoint number for next waypoint
            self.waypoint_num = self.waypoint_num + 1
        else:
            rospy.logwarn("Can't save Waypoint, No Transform Yet")
            
    #==========================================================================        
    def save_map(self):
        '''
        function to save map
        '''
        
        # Put rtabmap in localization mode so it does not continue to update map after we save it
        try:
            rtabmap_localization_mode = rospy.ServiceProxy('rtabmap/set_mode_localization',Empty())
            rtabmap_localization_mode()
            
            # save map file using map_server
            sts = subprocess.call('cd "%s"; rosrun map_server map_saver -f "%s"' % (self.path_map, self.file_map), shell=True)
            rospy.loginfo( "Save Map returned sts %d" % sts)
            
        except Exception as ex:
            rospy.loginfo("Save map crashed: %s" % str(ex))
        

        
    #==========================================================================
    def make_marker_arrow(self): 
        # method to create RVIZ marker
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.id = self.waypoint_num
        
        marker.type= marker.ARROW
        marker.action = marker.ADD
        
        marker.pose.position.x = self.tf[0][0]
        marker.pose.position.y = self.tf[0][1]
        marker.pose.position.z = 0.3

        marker.pose.orientation.x = 0.70711;
        marker.pose.orientation.y = 0;
        marker.pose.orientation.z = -0.70711;
        marker.pose.orientation.w = 0;
           
        marker.scale.x = 0.3
        marker.scale.y = 0.02
        marker.scale.z = 0.02
        
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        
        marker.lifetime = rospy.Duration()
        
        return marker

#==========================================================================
    def make_marker_text(self, text): 
        # method to create RVIZ marker
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.id = self.waypoint_num + 1000;
        
        marker.type= marker.TEXT_VIEW_FACING
        marker.action = marker.ADD
        marker.text = text
        
        marker.pose.position.x = self.tf[0][0]
        marker.pose.position.y = self.tf[0][1]
        marker.pose.position.z = 0.35
        
        marker.pose.orientation.x = 0;
        marker.pose.orientation.y = 0;
        marker.pose.orientation.z = 0;
        marker.pose.orientation.w = 1.0;
        marker.scale.z = 0.07
        
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        marker.lifetime = rospy.Duration()
        
        return marker
#==============================================================================
# main function
if __name__=="__main__":
    
    sts = 0
    rospy.loginfo("Running Mapper")
    try:
        # ROS initializzation
        rospy.init_node("mapper")
        mapper = Mapper()
        sts = mapper.run()

    except Exception as ex:
        rospy.loginfo("Mapper Crashed with exception: %s" % str(ex))
        sts = -1
    finally:
        rospy.loginfo("Mapper Finished")
        sys.exit(sts)





