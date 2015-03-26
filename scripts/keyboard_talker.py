#!/usr/bin/env python
#
# simple node to publish any pressed keyboard button
# except CTRL^C which kills the node
import rospy
import sys, select, termios, tty

from std_msgs.msg import String

def keyboard_talker():
    pub = rospy.Publisher('keypress_talker', String, queue_size=10)
    rospy.init_node('keyboard_publisher', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    rospy.loginfo("Keyboard publisher initialized")
    while not rospy.is_shutdown():
        key = getKey();
        if (key == '\x03'):
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
            break
        #rospy.loginfo(key)
        pub.publish(key)
        rate.sleep()
        
def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

if __name__ == '__main__':
    
    settings = termios.tcgetattr(sys.stdin)

    try:
        keyboard_talker()
    except rospy.ROSInterruptException:
        pass