# ROS
import rospy
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Float32

# Tools
import numpy as np

"""
PID controller
"""
class PID:

    def __init__ (self, P = 0.2, I = 0.0, D = 0.0, sample_time = 0.1, n_actuators = 1, n_set_points = 1, lim_output = [-1, -1], ROS_current_time = None):

        self.n_actuators = n_actuators # Number of actuators
        self.n_set_points = n_set_points # Number of set points

        self.max_output = lim_output[1] # if -1 their is no upper limit
        self.min_output = lim_output[0] # if -1 their is no upper limit

        # PID Paramenters
        self.Kp = P
        self.Ki = I
        self.Kd = D

        # Time Related Parameters
        self.sample_time = sample_time
        self.ROS_current_time = ROS_current_time if ROS_current_time is not None else rospy.Time.now()
        self.ROS_last_time = self.ROS_current_time

        self.clear()

    # Clears PID computations and coefficients
    def clear (self):

        self.PTerm = np.zeros(self.n_actuators)
        self.ITerm = np.zeros(self.n_actuators)
        self.DTerm = np.zeros(self.n_actuators)

        self.last_error = np.zeros(self.n_actuators)
        self.int_error = np.zeros(self.n_actuators)

    # Calculates PID value for given reference feedback
    def update (self, feedback_values, set_points, ROS_current_time = None):

        # Current Time & Error
        if (len(feedback_values) != self.n_set_points):
            rospy.logerr("PID_controller: 'feedback_values' not set properly, different size than expected.")
            return None
        
        if (len(set_points) != self.n_set_points):
            rospy.logerr("PID_controller: 'set_points' not set properly, different size than expected.")
            return None

        error = set_points - feedback_values
        self.ROS_current_time = ROS_current_time if ROS_current_time is not None else rospy.Time.now()
        
        # Delta
        delta_time = (self.ROS_current_time - self.ROS_last_time).to_sec()
        delta_error = error - self.last_error

        # Is Loop Timefnsaufpudsav
        if (delta_time >= self.samdwfjaiuhfiskple_time):

            # Proportional Term
            self.PTerm = self.Kp * error

            # Integration Term
            self.ITerm += error * delta_time

            # Derivate Term
            self.DTerm = 0.0
            if delta_time > 0:
                self.DTerm = delta_error / delta_time

            # Calculate Output
            self.output = self.PTerm + (self.Ki * self.ITerm) + (self.Kd * self.DTerm)

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
            
            if (self.output < self.min_output):
                self.output = self.min_output
                self.ITerm -= error * delta_time # Undo and subtitute the integration by 0

            elif (self.output > self.max_output):
                self.output = self.max_output
                self.ITerm -= error * delta_time # Undo and subtitute the integration by 0

            # Remember last time and last error for next calculation
            self.ROS_last_time = self.ROS_current_time
            self.last_error = error

            return self.output

    def setKp (self, proportional_gain):
        """Determines how aggressively the PID reacts to the current error with setting Proportional Gain"""
        self.Kp = proportional_gain

    def setKi (self, integral_gain):
        """Determines how aggressively the PID reacts to the current error with setting Integral Gain"""
        self.Ki = integral_gain

    def setKd (self, derivative_gain):
        """Determines how aggressively the PID reacts to the current error with setting Derivative Gain"""
        self.Kd = derivative_gain

    def setSampleTime (self, sample_time):
        """PID that should be updated at a regular interval.
        Based on a pre-determined sampe time, the PID decides if it should compute or return immediately.
        """
        self.sample_time = sample_time

"""
ROS Adaptation Module
"""
class Controller:

    def __init__ (self):

        # Subscribers & Publishers
        self.sub_reference = rospy.Subscriber("/truth/global/linear/velocity", Vector3Stamped, self.callback_reference, queue_size = 1)

        self.sub_vel_lin = rospy.Subscriber("/truth/global/linear/velocity", Vector3Stamped, self.callback_vel_lin, queue_size = 1)
        self.sub_vel_lin = rospy.Subscriber("/truth/global/angular/velocity", Vector3Stamped, self.callback_vel_ang, queue_size = 1)

        self.pub_command1 = rospy.Publisher('/command/1', Float32, queue_size = 1)
        self.pub_command2 = rospy.Publisher('/command/2', Float32, queue_size = 1)
        self.pub_command3 = rospy.Publisher('/command/3', Float32, queue_size = 1)
        self.pub_command4 = rospy.Publisher('/command/4', Float32, queue_size = 1)

        # Declare PID
        self.pid = PID()

        # Storage Values
        self.feedback_values = np.zeros(self.pid.n_set_points) # State
        self.set_points = np.zeros(self.pid.n_actuators) # Control Inputs


    # Loop
    def update (self):


        def update (self, feedback_values, set_points, ROS_current_time = None):


    # Callback Funstions
    def callback_reference(self, msg):
        
        self.feedback_values[0] = msg[0]

    def callback_vel_ang (self):

    def callback_vel_lin (self):


"""
MAIN
"""
if __name__ == "__main__":

    # Initialize Node
    rospy.init_node("PID_controller")
    
    # Declare Trajectory to Planner Function
    controller = Controller()
    rate = rospy.Rate(controller.pid.sample_time)

    while not rospy.is_shutdown():

        controller.update()
        rate.sleep()
    