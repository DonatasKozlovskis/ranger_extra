#!/usr/bin/env python

#######################
# Import required Python code.
import rospy

import sys

# ROS messages
from std_msgs.msg import Header
from std_msgs.msg import String

###############################################
#
#   Class for the MapperControl.py ROS node
class MapperControl():
    #
    # Constructor
    #
    def __init__(self):
    
        # Get the ~private namespace parameters from command line or launch file.
        self._topic_app_frame_add = rospy.get_param('~app_topic_frame_add', 'frame_publisher_topic');
        self._topic_keypress = rospy.get_param('~keypress_talker', 'keypress_talker');
        self._topic_robotouch = rospy.get_param('~robot_touch_talker', 'robot_touch_talker');
        
        # Subscribers
        rospy.Subscriber(self._topic_app_frame_add, String,     self.app_frame_add_callback)
        rospy.Subscriber(self._topic_keypress,      String,     self.key_callback)
        rospy.Subscriber(self._topic_robotouch,     String,     self.robotouch_callback)
        
        # Publisher for mapper to execute commands
        self._command_pub = rospy.Publisher("mapper_control", Header, queue_size=5)
        
        self.rate = rospy.Rate(10.0)    #10 Hz
                        
        self.keyboard_frame_counter = 0;
    
    #==========================================================================
    def run(self):
        '''
        main run loop
        '''
        rospy.loginfo("Mapper Control Ready")

        while not rospy.is_shutdown():        
            self.rate.sleep()
        return 0

    
    #==========================================================================
    def app_frame_add_callback(self,msg):
        rospy.loginfo("Heard App button press %s", msg.data)

        controlMessage = Header()
        #command to add waypoint;
        controlMessage.seq = 1;
        controlMessage.frame_id = msg.data;
        self._command_pub.publish(controlMessage)
        
    #==========================================================================
    def key_callback(self,msg):
        '''
        function to handle keyboard buttons
        '''
        key = str(msg.data)
        
        #rospy.loginfo("Heard keypress %s", key)
                    
        if key == "a":
            self.keyboard_frame_counter = self.keyboard_frame_counter + 1;

            controlMessage = Header()
            #command to add waypoint;
            controlMessage.seq = 1;
            controlMessage.frame_id = 'Frame_' + str(self.keyboard_frame_counter);
            self._command_pub.publish(controlMessage)

        if key == "d":
            controlMessage = Header()
            #command to delete waypoint;
            #deleting last waypoint
            controlMessage.seq = 2;
            controlMessage.frame_id = 'Frame_' + str(self.keyboard_frame_counter);
            self._command_pub.publish(controlMessage)
            
        if key == "\x13": #CTRL+ S button code           
            controlMessage = Header()
            #command to save map;
            controlMessage.seq = 3;
            self._command_pub.publish(controlMessage)

        
    #==========================================================================
    def robotouch_callback(self,msg):
        '''
        function to robot touch messages
        '''
        key = str(msg.data);

    #==========================================================================

#==============================================================================
# main function
if __name__=="__main__":
    
    sts = 0
    rospy.loginfo("Running Mapper Control")

    try:
        # ROS initializzation
        rospy.init_node("mapper_control")

        mapperControl = MapperControl()
        sts = mapperControl.run()

    except Exception as ex:
        rospy.loginfo("Mapper Control Crashed with exception: %s" % str(ex))
        sts = -1
        
    finally:
        rospy.loginfo("Mapper Control Finished")
        sys.exit(sts)





