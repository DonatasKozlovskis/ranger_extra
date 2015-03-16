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
from std_msgs.msg import String

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
    
        # Get the relative parameters from command line or launch file.
        self.path_map = rospy.get_param("path_map", "maps")
        self.path_waypoint =  rospy.get_param("path_waypoint", "maps")
        
        self.file_map = rospy.get_param("file_map", "map")
        self.file_waypoint = rospy.get_param("file_waypoint", "waypoint.txt")
        
        self.full_path_waypoint = os.path.join(self.path_waypoint, self.file_waypoint)
        
        # ensure that directories exist, if not create
        dir = os.path.dirname(self.path_map)
        if not os.path.exists(dir):
            os.makedirs(dir)

        dir = os.path.dirname(self.path_waypoint)
        if not os.path.exists(dir):
            os.makedirs(dir)          
                            
        # Subscriber
        rospy.Subscriber("keypress_button", String, self.key_callback)

        # need the transform from  /map to /base_footprint
        self.listener = tf.TransformListener()
        
        self.rate = rospy.Rate(10.0) #10 Hz
        
        self.tf = None          # not seen a transform yet
        self.waypoint_num = 0   # waypoint enumerator
                
        # give tf a chance to queue up some transforms
        rospy.sleep(3)
    
    #==========================================================================
    def run(self):
        '''
        main run loop
        '''
        rospy.loginfo("Mapper Ready")

        while not rospy.is_shutdown():
            transform_ok = False
            try:
                # check for transform
                (trans, rot) = self.listener.lookupTransform('map', 'base_link', rospy.Time(0))

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
    def key_callback(self,msg):
        '''
        function to handle keyboar buttons
        '''
        key = str(msg.data)
        
        rospy.loginfo("Heard keypress %s", key)
                    
        if key == "a":            
            rospy.loginfo("Heard keypress %s to add waypoint!", key)
            rospy.loginfo("Adding waypoint to %s", self.full_path_waypoint)
            
        if key == "\x13": #CTRL+ S button code           
            rospy.loginfo("Heard keypress CTRL+S to save map!")
            rospy.loginfo("Saving map to %s", self.path_map)

            
    #==========================================================================        
    def save_waypoint(self):
        '''
        function to save waypoint to file
        '''
        
        if self.tf != None: # do we have a valid transform to use    
            #save in the waypoint file and create a visualization marker
            #self.marker_pub.publish(self.make_arrow_marker(self.tf, self.waypoint_num))
                    
            rospy.loginfo("Saving waypoint %d: %s, %s" %(self.waypoint_num, str(self.tf[0]), str(self.tf[1])))

            # save waypoint pose in file
            with open(self.full_path_waypoint,'a+') as wpfh:
                wpfh.write("%d: %s\n" % (self.waypoint_num, str(self.tf)))

            rospy.loginfo("Waypoint %d saved." % self.waypoint_num)

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
        rtabmap_localization_mode = rospy.ServiceProxy('rtabmap/set_mode_localization',Empty())
        rtabmap_localization_mode()
        
        # save map file using map_server
        sts = subprocess.call('cd "%s"; rosrun map_server map_saver -f "%s"' % (self.path_map, self.file_map), shell=True)

        rospy.loginfo( "Save Map returned sts %d" % sts)
        
    #==========================================================================
    def make_marker(self):
        # method to create RVIZ marker
        rospy.loginfo("make_marker Not implemented")

#==============================================================================
# main function
if __name__=="__main__":
    
    sts = 0
    
    rospy.loginfo("Running Mapper")

    try:
        # ROS initializzation
        rospy.init_node('mapper')

        mapper = Mapper()
        sts = mapper.run()

    except Exception as ex:
        rospy.loginfo("Mapper Crashed with exception: %s" % str(ex))
        sts = -1
        
    finally:
        rospy.loginfo("Mapper Finished")
        sys.exit(sts)





