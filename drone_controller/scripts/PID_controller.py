#!/usr/bin/env python

# Copyright (c) 2020 Authors:
#   - Félix Martí Valverde <martivalverde@hotmail.com>
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

# ROS Messages
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu

# ROS
import rospy
from dynamic_reconfigure.server import Server
from drone_controller.cfg import PID_tunningConfig

# Tools
import numpy as np
import yaml


"""
PID controller
"""
class PID:

    def __init__ (self, sample_time = 0.1, N = 1, lim_output = [-1, -1], ROS_current_time = None):

        self.N = N # Number of degrees
        
        # NOTE: their is no upper limit if "lim_output" equal -1, 
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

        # Storage
        self.output = np.zeros(self.N) # Motor 1, Motor 2, ...
        self.error = np.zeros(self.N) # Thrust Roll Pitch Yaw
        self.feedback_values = np.zeros(self.N - 1) # Thrust Roll Pitch Yaw
        self.set_points = np.zeros(self.N) # Thrust Roll Pitch Yaw

    # Clears PID computations and coefficients
    def clear (self):

        self.PTerm = np.zeros(self.N)
        self.ITerm = np.zeros(self.N)
        self.DTerm = np.zeros(self.N)

        self.gyroscopic_orientation = np.zeros(self.N - 1) # Roll Pitch Yaw
    
    def obtainPIDTerms (self, delta_time):

        # Update Gyroscopic Orientation
        self.gyroscopic_orientation += self.feedback_values[1:4] * delta_time
        if (np.any(abs(self.gyroscopic_orientation) > np.pi)):
            for i in np.where(abs(self.gyroscopic_orientation > np.pi)):
                self.gyroscopic_orientation[i] -= np.sign(self.gyroscopic_orientation[i]) * 2 * np.pi

        # P Term (Current Error)
        self.PTerm[0] = self.set_points[0] - self.feedback_values[0]
        self.PTerm[1:4] = self.set_points[1:4] - self.gyroscopic_orientation

        # I Term (Integral of the Error)
        self.ITerm[0] += self.PTerm[0] * delta_time
        self.ITerm[1:4] += - self.PTerm[1:4] * delta_time

        # D Term (Derivative of the Error)
        self.DTerm[0] = 0 # Not interested in this value
        self.DTerm[1:4] = - self.feedback_values[1:4]
            
    # Calculates PID value for given reference feedback
    def update (self, feedback_values, set_points, ROS_current_time = None):

        # Save Values
        self.feedback_values = feedback_values # Thrust, Roll, Pitch, Yaw
        self.set_points = set_points # Thrust, Roll, Pitch, Yaw

        # Delta Time
        self.ROS_current_time = ROS_current_time if ROS_current_time is not None else rospy.Time.now()
        delta_time = (self.ROS_current_time - self.ROS_last_time).to_sec()

        # Calculate Output
        self.obtainPIDTerms(delta_time)
        self.error = (self.Kp * self.PTerm) + (self.Ki * self.ITerm) + (self.Kd * self.DTerm)

        # NOTE: this process returns a matrix but we want only the first row, an array
        self.output = np.matmul(self.A, self.B * self.error)
        self.output = np.squeeze(np.array(self.output))
    
        """
        Anti Windup
        
        Integral windup, also known as integrator windup or reset windup,
        refers to the situation in a PID feedback controller where
        a large change in set_points occurs (say a positive change)
        and the integral terms accumulates a significant error
        during the rise (windup), thus overshooting and continuing
        to increase as this accumulated error is unwound
        (offset by errors in the other direction).
        The specific problem is the excess overshooting.
        """
        
        if (np.any(self.output < self.min_output)):
            for i in np.where(self.output < self.min_output):
                self.output[i] = self.min_output
                self.ITerm[i] -= self.PTerm * delta_time # Undo the integration

        if (np.any(self.output > self.max_output)):
            for i in np.where(self.output > self.max_output):
                self.output[i] = self.max_output
                self.ITerm[i] -= self.PTerm * delta_time # Undo the integration

        # Remember for the next calculation
        self.ROS_last_time = self.ROS_current_time

        return self.output

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

    # Debug
    def debug (self):

        print("\n{:^20}".format("OUTPUTS"))

        print("motor1:{:>13.2f}".format(self.output[0]))
        print("motor2:{:>13.2f}".format(self.output[1]))
        print("motor3:{:>13.2f}".format(self.output[2]))
        print("motor4:{:>13.2f}".format(self.output[3]))

        print("\n{:^20}".format("ERROR"))

        print("thrust:{:>13.2f}".format(self.error[0]))
        print("roll:{:>15.2f}".format(self.error[1]))
        print("pitch:{:>14.2f}".format(self.error[2]))
        print("yaw:{:>16.2f}".format(self.error[3]))

        print("\n{:^20}".format("SET POINTS"))

        print("thrust:{:>13.2f}".format(self.set_points[0]))
        print("roll:{:>15.2f}".format(self.set_points[1]))
        print("pitch:{:>14.2f}".format(self.set_points[2]))
        print("yaw:{:>16.2f}".format(self.set_points[3]))

        print("\n{:^20}".format("FEEDBACK"))

        print("thrust:{:>13.2f}".format(self.feedback_values[0]))
        print("roll:{:>15.2f}".format(self.feedback_values[1]))
        print("pitch:{:>14.2f}".format(self.feedback_values[2]))
        print("yaw:{:>16.2f}".format(self.feedback_values[3]))

        print("====================")


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
        self.pid = PID(0.05, 4) # 20 Hz

        k = p["k"]
        lr = p["lr"]
        lf = p["lf"]
        s = p["s"]
        b = p["b"]

        # error -> output
        A = np.matrix( [[k      , k     , k     , k     ],  # Thrust
                        [-lf*k  , lr*k  , lr*k  , -lf*k ],  # Roll  (X)
                        [-s*k   , -s*k  , s*k   , s*k   ],  # Pitch (Y)
                        [b      , -b    , b     , -b    ]]) # Yaw   (Z)
        A = np.linalg.inv(A)
        self.pid.setAMatrix(A)

        i = p["inertial"]
        
        B = np.array([1, i["xx"], i["yy"], i["zz"]])
        self.pid.setBVector(B)

        self.mass = p["mass"]

        # Storage Values
        self.feedback_values = np.zeros(self.pid.N) # State
        self.set_points = np.zeros(self.pid.N) # Control Inputs

        # Limit Inputs 
        self.lim_roll = np.pi / 4
        self.lim_pitch = np.pi / 4
        self.lim_yaw = np.pi / 4
        self.lim_thrust = [-5, 10]

        # Set Kp, Ki, Kd Arrays
        # NOTE: Thrust must NOT be treated as a PID controller, it is only use to solve the equation, 
        # thus KpT = 1, and KiT = KdT = 0
        Kp = np.array([1, 2, 2, 2]) # Thrust, Roll, Pitch, Yaw
        self.pid.setKp(Kp)

        Ki = np.array([0, 0.1, 0.1, 0.1]) # Thrust, Roll, Pitch, Yaw
        self.pid.setKi(Ki)

        Kd = np.array([0, 0, 0, 0]) # Thrust, Roll, Pitch, Yaw
        self.pid.setKd(Kd)

    # Loop
    def update (self, debug = False):
        output = self.pid.update(self.feedback_values, self.set_points)
        self.publish(output)

        if debug: 
            self.pid.debug()

    # Callback Functions
    def callback_feedback(self, msg):
        
        # Angular Valocities - Local Frame
        self.feedback_values[0] = msg.linear_acceleration.z * self.mass
        self.feedback_values[1] = msg.angular_velocity.x
        self.feedback_values[2] = msg.angular_velocity.y
        self.feedback_values[3] = msg.angular_velocity.z

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
        if (msg.data > self.lim_thrust[1]):
            self.set_points[0] = self.lim_thrust[1]
        elif (msg.data < self.lim_thrust[0]):
            self.set_points[0] = self.lim_thrust[0]
        else:
            self.set_points[0] = msg.data

    # Publish State
    def publish (self, output):

        self.pub_command1.publish(output[0])
        self.pub_command2.publish(output[1])
        self.pub_command3.publish(output[2])
        self.pub_command4.publish(output[3])

    # Dynamic Reconfigure
    def callback_dynamic_reconfigure(self, config, level):
            
        rospy.loginfo("PID Controller: Logging new dynamic parameters.")

        Kp = np.array([config.Kp_Thrust, config.Kp_Roll, config.Kp_Pitch, config.Kp_Yaw])
        self.pid.setKp(Kp)

        Ki = np.array([config.Ki_Thrust, config.Ki_Roll, config.Ki_Pitch, config.Ki_Yaw])
        self.pid.setKi(Ki)

        Kd = np.array([config.Kd_Thrust, config.Kd_Roll, config.Kd_Pitch, config.Kd_Yaw])
        self.pid.setKd(Kd)
        
        return config


"""
MAIN
"""
if __name__ == "__main__":

    # Initialize Node
    rospy.init_node("PID_controller")
    
    # Declare Trajectory to Planner Function
    controller = Controller("/home/felix/Desktop/ws_drone/src/drone_sw/drone_controller/conf/params_drone.yaml")
    rate = rospy.Rate(1 / controller.pid.sample_time) # Hz

    # Dynamic Parameters
    srv = Server(PID_tunningConfig, controller.callback_dynamic_reconfigure)

    # Loop At Controller Rate
    while not rospy.is_shutdown():

        controller.update(debug = True)
        rate.sleep()