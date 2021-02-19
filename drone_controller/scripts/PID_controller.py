#!/usr/bin/env python

# ROS
import rospy
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Float32

# Tools
import numpy as np
import yaml


"""
PID controller
"""
class PID:

    def __init__ (self, sample_time = 0.1, N = 1, lim_output = [-1, -1], ROS_current_time = None):

        self.N = N # Number of degrees

        self.max_output = lim_output[1] # if -1 their is no upper limit
        self.min_output = lim_output[0] # if -1 their is no upper limit

        # PID Paramenters
        self.Kp = np.zeros(self.N)
        self.Ki = np.zeros(self.N)
        self.Kd = np.zeros(self.N)

        # Time Related Parameters
        self.sample_time = sample_time
        self.ROS_current_time = ROS_current_time if ROS_current_time is not None else rospy.Time.now()
        self.ROS_last_time = self.ROS_current_time

        self.clear()

        # error -> output
        self.A = np.eye(self.N)
        self.B = np.ones(self.N)


    # Clears PID computations and coefficients
    def clear (self):

        self.PTerm = np.zeros(self.N)
        self.ITerm = np.zeros(self.N)
        self.DTerm = np.zeros(self.N)

        self.last_error = np.zeros(self.N)
        self.int_error = np.zeros(self.N)


    # Calculates PID value for given reference feedback
    def update (self, feedback_values, set_points, ROS_current_time = None):

        """
        # Current Time & Error
        if (len(feedback_values) != self.N):
            rospy.logerr("PID_controller: 'feedback_values' not set properly, different size than expected.")
            return None
        
        if (len(set_points) != self.N):
            rospy.logerr("PID_controller: 'set_points' not set properly, different size than expected.")
            return None
        """

        self.ROS_current_time = ROS_current_time if ROS_current_time is not None else rospy.Time.now()
        delta_time = (self.ROS_current_time - self.ROS_last_time).to_sec()

        # Derivate Term
        self.DTerm = feedback_values

        # Proportional Term
        self.PTerm += feedback_values * delta_time
        
        # Integration Term
        self.ITerm = self.PTerm * delta_time

        # Calculate Output
        output = (self.Kp * self.PTerm) + (self.Ki * self.ITerm) + (self.Kd * self.DTerm)
    
        """
        Anti Windup
        
        Integral windup, also known as integrator windup or reset windup,
        refers to the situation in a PID feedback controller where
        a large change in set_point occurs (say a positive change)
        and the integral terms accumulates a significant error
        during the rise (windup), thus overshooting and continuing
        to increase as this accumulated error is unwound
        (offset by errors in the other direction).
        The specific problem is the excess overshooting.
        """
        
        if (output < self.min_output):
            output = self.min_output
            self.ITerm -= feedback_values * delta_time # Undo by subtituting the integration by 0

        elif (output > self.max_output):
            output = self.max_output
            self.ITerm -= feedback_values * delta_time # Undo by subtituting the integration by 0

        # Remember last time and last error for next calculation
        self.ROS_last_time = self.ROS_current_time

        return output


    def setKp (self, proportional_gain):
        """ Determines how aggressively the PID reacts to the current error with setting Proportional Gain """
        self.Kp = proportional_gain

    def setKi (self, integral_gain):
        """ Determines how aggressively the PID reacts to the current error with setting Integral Gain """
        self.Ki = integral_gain

    def setKd (self, derivative_gain):
        """ Determines how aggressively the PID reacts to the current error with setting Derivative Gain """
        self.Kd = derivative_gain

    def setSampleTime (self, sample_time):
        """ PID that should be updated at a regular interval. Based on a pre-determined sampe time, the PID decides if it should compute or return immediately. """
        self.sample_time = sample_time

    def setAMatrix (self, A):
        """ This matrix is used to convert the error (feedback - desired) in the PID to output values for the actuators. """
        self.A = A

    def setBVector (self, B):
        """ This vector is used to convert the error (feedback - desired) in the PID to output values for the actuators. """
        self.B = B


"""
ROS Adaptation Module
"""
class Controller:

    def __init__ (self, _file):

        # Import Parameters Drone
        p = yaml.load(_file, Loader=yaml.FullLoader)

        # Subscribers & Publishers
        self.sub_roll = rospy.Subscriber("/input/roll", Float32, self.callback_roll, queue_size = 1)
        self.sub_pitch = rospy.Subscriber("/input/pitch", Float32, self.callback_pitch, queue_size = 1)
        self.sub_yaw = rospy.Subscriber("/input/yaw", Float32, self.callback_yaw, queue_size = 1)
        self.sub_thrust = rospy.Subscriber("/input/thrust", Float32, self.callback_thrust, queue_size = 1)
        
        self.sub_vel_ang = rospy.Subscriber("/truth/local/angular/velocity", Vector3Stamped, self.callback_feedback, queue_size = 1)

        self.pub_command1 = rospy.Publisher('/command/1', Float32, queue_size = 1)
        self.pub_command2 = rospy.Publisher('/command/2', Float32, queue_size = 1)
        self.pub_command3 = rospy.Publisher('/command/3', Float32, queue_size = 1)
        self.pub_command4 = rospy.Publisher('/command/4', Float32, queue_size = 1)

        # Declare PID Module
        self.pid = PID()

        # error -> output
        A = np.matrix( [[p.k      , p.k     , p.k     , p.k     ],
                        [p.L*p.k    , 0     , - p.L*p.k , 0     ],
                        [0      , p.L*p.k   , 0     , -p.L*p.k  ],
                        [p.b      , -p.b    , p.b     , -p.b    ]])
        A = np.linalg.inv(A)
        self.pid.setAMatrix(A)
        
        B = np.array([1, p.Ixx, p.Iyy, p.Izz])
        self.pid.setBVector(B)

        # Storage Values
        self.feedback_values = np.zeros(self.pid.N) # State
        self.set_points = np.zeros(self.pid.N) # Control Inputs

        # Limit Inputs 
        self.lim_roll = np.pi/4
        self.lim_pitch = np.pi/4
        self.lim_yaw = np.pi/4
        self.lim_thrust = 10

    # Numerical Integration - Trapezoidal Theorem
    def integrate(self, fa, fb, delta_time):
        return delta_time * (fa + fb) / 2

    # Loop
    def update (self):
        output = self.pid.update(self.feedback_values, self.set_points)
        self.publish(output)

    # Callback Functions
    def callback_feedback(self, msg):
        
        # Angular Valocities - Local Frame
        self.feedback_values[0] = msg[0]
        self.feedback_values[1] = msg[1]
        self.feedback_values[2] = msg[2]

    def callback_roll (self, msg):
        if (abs(msg) > self.lim_roll):
            self.set_points[1] = np.sign(msg) * self.lim_roll

    def callback_pitch (self, msg):
        if (abs(msg) > self.lim_pitch):
            self.set_points[2] = np.sign(msg) * self.lim_pitch
    
    def callback_yaw (self, msg):
        if (abs(msg) > self.lim_yaw):
            self.set_points[3] = np.sign(msg) * self.lim_yaw
    
    def callback_thrust (self, msg):
        if (msg > self.lim_thrust):
            self.set_points[0] = self.lim_thrust
        elif (msg < 0):
            self.set_points[0] = 0

    # Publish State
    def publish (self, output):

        self.pub_command1.publish(output[0])
        self.pub_command2.publish(output[1])
        self.pub_command3.publish(output[2])
        self.pub_command4.publish(output[3])


"""
MAIN
"""
if __name__ == "__main__":

    # Initialize Node
    rospy.init_node("PID_controller")
    
    # Declare Trajectory to Planner Function
    controller = Controller("/home/felix/Desktop/ws_drone/src/drone_sw/drone_controller/conf/params_drone.yaml")
    rate = rospy.Rate(1 / controller.pid.sample_time) # Hz

    # Loop At Controller Rate
    while not rospy.is_shutdown():

        controller.update()
        rate.sleep()