#!/usr/bin/env python
import signal
import time
import rospy
import numpy as np
from sensor_msgs.msg import JointState
from dynamic_reconfigure.server import Server
from ahand_controllers.cfg import gains_pd_paramConfig

from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension


class State:

    def __init__(self, position=None, velocity=None):
        self.position = position
        self.velocity = velocity
        self.joint_name_prefix_offset = None
        self.num_joints = 16
        self.joint_idx_mapping = [0]*self.num_joints

    def update(self, data):
        if data is None:
            return
        if data.position is None:
            return
        if self.joint_name_prefix_offset is None:
            first_joint_name_ = data.name[0]
            self.joint_name_prefix_offset = first_joint_name_.find('joint_') + len('joint_')
            for i, name in enumerate(data.name):
                self.joint_idx_mapping[i] = int(name[self.joint_name_prefix_offset:])

        if self.position is None:
            self.position = np.empty(self.num_joints, dtype=float)
            self.velocity = np.empty(self.num_joints, dtype=float)

        for i, j in enumerate(self.joint_idx_mapping):
            self.position[j] = data.position[i]
            self.velocity[j] = data.velocity[i]


class JointImpedanceController:

    def __init__(self):
        # cfg reconfigure
        self.kp = np.zeros(16)
        self.kd = np.zeros(16)
        rospy.init_node("gains_pd", anonymous=True)
        self.cfg_server = Server(gains_pd_paramConfig, self.gains_callback)

        # state callback SUBSCRIBER
        rospy.Subscriber('/ahand/joint_states', JointState, self.joint_state_callback)
        self.joint_state = State()
        self.joint_filtered_position = np.zeros(16)
        self.joint_filtered_velocity = np.zeros(16)


        # action msg PUBLISHER
        self.action_msg = Float32MultiArray()
        self.action_msg.layout.dim.append(MultiArrayDimension())
        self.action_msg.layout.dim[0].size = 16
        self.action_msg.layout.data_offset = 0
        self.action_msg.data = np.zeros(16)
        self.pub_torques = rospy.Publisher('/ahand/command', Float32MultiArray, queue_size=1)

        self.joint_des_position = np.array([0.0,  35.0, 35.0, 35.0,
                                            0.0,  35.0, 35.0, 35.0,
                                            0.0,  35.0, 35.0, 35.0,
                                           40.0,   0.0, 35.0, 45.0])*np.pi/180.0

    @staticmethod
    def exponential_smoothing(current_raw_value, last_smoothed_value, alpha):
        return alpha * current_raw_value + (1.0 - alpha) * last_smoothed_value

    def update(self):
        if self.joint_state.position is None:
            return
        self.joint_filtered_position = JointImpedanceController.exponential_smoothing(self.joint_state.position, self.joint_filtered_position, 0.2)
        self.joint_filtered_velocity = JointImpedanceController.exponential_smoothing(self.joint_state.velocity, self.joint_filtered_velocity, 0.2)

        tau_cmd = self.kp*(self.joint_des_position - self.joint_filtered_position) - self.kd*self.joint_filtered_velocity

        self.action_msg.data = tau_cmd
        self.pub_torques.publish(self.action_msg)

    def joint_state_callback(self, data):
        self.joint_state.update(data)

    def gains_callback(self, config, level):
        # finger 1
        self.kp[0] = config['p00']
        self.kp[1] = config['p01']
        self.kp[2] = config['p02']
        self.kp[3] = config['p03']
        # finger 2
        self.kp[4] = config['p10']
        self.kp[5] = config['p11']
        self.kp[6] = config['p12']
        self.kp[7] = config['p13']
        # finger 3
        self.kp[8] = config['p20']
        self.kp[9] = config['p21']
        self.kp[10] = config['p22']
        self.kp[11] = config['p23']
        # finger 4
        self.kp[12] = config['p30']
        self.kp[13] = config['p31']
        self.kp[14] = config['p32']
        self.kp[15] = config['p33']

        # finger 1
        self.kd[0] = config['d00']
        self.kd[1] = config['d01']
        self.kd[2] = config['d02']
        self.kd[3] = config['d03']
        # finger 2
        self.kd[4] = config['d10']
        self.kd[5] = config['d11']
        self.kd[6] = config['d12']
        self.kd[7] = config['d13']
        # finger 3
        self.kd[8] = config['d20']
        self.kd[9] = config['d21']
        self.kd[10] = config['d22']
        self.kd[11] = config['d23']
        # finger 4
        self.kd[12] = config['d30']
        self.kd[13] = config['d31']
        self.kd[14] = config['d32']
        self.kd[15] = config['d33']
        return config


def signal_handler(sig, frame, flag):
    flag[0] = True


if __name__ == "__main__":
    terminated = [False]

    signal.signal(signal.SIGINT, lambda sig, frame : signal_handler(sig, frame, terminated))
    jointImpedanceController = JointImpedanceController()

    time.sleep(2.0)
    print('start')
    while not terminated[0]:
        jointImpedanceController.update()
        time.sleep(0.001)

    print('joint impedance controller finished!')
