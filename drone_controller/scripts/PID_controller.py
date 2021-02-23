#!/usr/bin/env python

# ROS
import rospy
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu

# Tools
import numpy as np
import yaml


"""
PID controller
"""
class PID:

    def __init__ (self, sample_time = 0.1, N = 1, lim_output = [-1, -1], ROS_current_time = None):

        self.N = N # Number of degrees
        
        # NOTE: their is no upper limit if lim_output equal -1, 
        if (lim_output[1] == -1):
            self.max_output = 1e10
        else: 
            self.max_output = lim_output[1]
        if (lim_output[0] == -1):
            self.min_output = -1e10
        else: 
            self.min_output = lim_output[0]
        
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

        error = feedback_values - set_points

        # Derivate Term
        self.DTerm = error

        # Proportional Term
        self.PTerm += error * delta_time
        
        # Integration Term
        self.ITerm = self.PTerm * delta_time

        # Calculate Output
        error = (self.Kp * self.PTerm) + (self.Ki * self.ITerm) + (self.Kd * self.DTerm)

        # NOTE: this process returns a matrix but we want only the first row, an array
        output = np.matmul(self.A, self.B * error)
        output = np.squeeze(np.array(output))
    
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
        
        if (np.any(output < self.min_output)):
            for i in np.where(output < self.min_output):
                output[i] = self.min_output
                self.ITerm[i] -= self.PTerm[i] * delta_time # Undo by subtituting the integration by 0

        if (np.any(output > self.max_output)):
            for i in np.where(output > self.max_output):
                output[i] = self.max_output
                self.ITerm[i] -= self.PTerm[i] * delta_time # Undo by subtituting the integration by 0

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
        _file = open(_file, "r")
        p = yaml.load(_file, Loader=yaml.FullLoader)

        # Subscribers & Publishers
        self.sub_roll = rospy.Subscriber("/input/roll", Float32, self.callback_roll, queue_size = 1)
        self.sub_pitch = rospy.Subscriber("/input/pitch", Float32, self.callback_pitch, queue_size = 1)
        self.sub_yaw = rospy.Subscriber("/input/yaw", Float32, self.callback_yaw, queue_size = 1)
        self.sub_thrust = rospy.Subscriber("/input/thrust", Float32, self.callback_thrust, queue_size = 1)
        
        self.sub_imu = rospy.Subscriber("/imu", Imu, self.callback_feedback, queue_size = 1)

        self.pub_command1 = rospy.Publisher("/command/1", Float32, queue_size = 1)
        self.pub_command2 = rospy.Publisher("/command/2", Float32, queue_size = 1)
        self.pub_command3 = rospy.Publisher("/command/3", Float32, queue_size = 1)
        self.pub_command4 = rospy.Publisher("/command/4", Float32, queue_size = 1)

        # Declare PID Module
        self.pid = PID(0.005, 4)

        k = p["k"]
        lr = p["lr"]
        lf = p["lf"]
        s = p["s"]
        b = p["b"]

        # error -> output
        A = np.matrix( [[k      , k     , k     , k     ],
                        [-lf*k  , lr*k  , lr*k  , -lf*k ],
                        [-s*k   , -s*k  , s*k   , s*k   ],
                        [b      , -b    , b     , -b    ]])
        A = np.linalg.inv(A)
        self.pid.setAMatrix(A)

        i = p["inertial"]
        
        B = np.array([1, i["xx"], i["yy"], i["zz"]])
        self.pid.setBVector(B)

        # Storage Values
        self.feedback_values = np.zeros(self.pid.N) # State
        self.set_points = np.zeros(self.pid.N) # Control Inputs

        # Limit Inputs 
        self.lim_roll = np.pi / 4
        self.lim_pitch = np.pi / 4
        self.lim_yaw = np.pi / 4
        self.lim_thrust = 10

    # Loop
    def update (self):
        output = self.pid.update(self.feedback_values, self.set_points)
        self.publish(output)

    # Callback Functions
    def callback_feedback(self, msg):
        
        # Angular Valocities - Local Frame
        self.feedback_values[0] = msg.angular_velocity.x
        self.feedback_values[1] = msg.angular_velocity.y
        self.feedback_values[2] = msg.angular_velocity.z

    def callback_roll (self, msg):
        if (abs(msg.data) > self.lim_roll):
            self.set_points[1] = np.sign(msg.data) * self.lim_roll
        else: 
            self.set_points[1] = msg.data

    def callback_pitch (self, msg):
        if (abs(msg.data) > self.lim_pitch):
            self.set_points[2] = np.sign(msg.data) * self.lim_pitch
        else:
            self.set_points[2] = msg.data
    
    def callback_yaw (self, msg):
        if (abs(msg.data) > self.lim_yaw):
            self.set_points[3] = np.sign(msg.data) * self.lim_yaw
        else:
            self.set_points[3] = msg.data
    
    def callback_thrust (self, msg):
        if (msg.data > self.lim_thrust):
            self.set_points[0] = self.lim_thrust
        elif (msg.data < 0):
            self.set_points[0] = 0
        else:
            self.set_points[0] = msg.data

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