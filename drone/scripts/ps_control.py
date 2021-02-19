#!/usr/bin/env python

# ROS Tools:
import rospy
from std_msgs.msg import Float32

# Tools:
import pygame

class PSControl:

    def __init__ (self):

        # Init PyGame & JoySticks
        pygame.init()
        pygame.joystick.init()

        # If the number of Joysticks connected is superior to 1
        # block the system from running as a security measure.
        if pygame.joystick.get_count() > 1:
            rospy.logerr("PS Controller: More than one controller connected, please use only one of them.")
            self.operational = False
            return None

        # If the number of Joysticks connected is inferior to 1
        # block the system from running as a security measure.
        elif pygame.joystick.get_count() != 1:
            rospy.logerr("PS Controller: No controller detected.")
            self.operational = False
            return None

        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()

        self.states = list()

        # ROS Publisher
        pub = rospy.Publisher("/input/roll", Float32, queue_size = 1)
        self.states.append({"pub": pub, "state": 0, "axis": 0})

        pub = rospy.Publisher("/input/pitch", Float32, queue_size = 1)
        self.states.append({"pub": pub, "state": 0, "axis": 1})

        pub = rospy.Publisher("/input/yaw", Float32, queue_size = 1)
        self.states.append({"pub": pub, "state": 0, "axis": 2})
        
        pub = rospy.Publisher("/input/thrust", Float32, queue_size = 1)
        self.states.append({"pub": pub, "state": 0, "axis": 3})

        # Status
        self.operational = True


    def read_axes (self):

        pygame.event.get()
        
        for i in range(len(self.states)):
            axis = self.joystick.get_axis(self.states[i]["axis"])
            self.states[i]["state"] = axis


    def run (self):

        self.read_axes()
        self.publish_commands()


    def publish_commands(self):

        for i in range(len(self.states)):
            self.states[i]["pub"].publish(self.states[i]["state"])


if __name__ == "__main__":
    
    # Init Node
    rospy.init_node("ps_control")

    # Init Publisher Class
    ps_control = PSControl()
    
    while not rospy.is_shutdown() and ps_control.operational:
        ps_control.run()
