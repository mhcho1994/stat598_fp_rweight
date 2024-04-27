# import public modules
import numpy as np
from scipy import integrate

from controllers.load_parameters import get_controller_parameters, get_miscellaneous_constants

# get constants
const   =   get_miscellaneous_constants()

class geometric_SE3_tracking():

    # Geometric Tracking Control on SE(3)
    # Follow the reference: [2010 Lee] Geometric Tracking Control of a Quadrotor UAV on SE(3)
    # to be updated
    
    # __init__
    def __init__(self, model = "parrotmambo"):
        
        ctrl_parameter  =   get_controller_parameters(model)

        # assign parameters from the dictionary
        # inertial parameters
        self.mass   =   ctrl_parameter['mass']                                                              # [kg] mass
        self.J      =   np.array([[ctrl_parameter['Ixx'], ctrl_parameter['Ixy'], ctrl_parameter['Ixz']],
                                  [ctrl_parameter['Ixy'], ctrl_parameter['Iyy'], ctrl_parameter['Iyz']],
                                  [ctrl_parameter['Ixz'], ctrl_parameter['Iyz'], ctrl_parameter['Izz']]])   # [kg*m^2] inertia
        self.Jinv   =   np.linalg.inv(self.J)       # [1/kg/m^2] inverse of inertia

        # geometric parameters
        self.rotor_pos  =   np.empty((ctrl_parameter['num_rotors'],3), dtype=np.float64)

        for index, (key, value) in enumerate(ctrl_parameter['rotor_config'].items()):
            self.rotor_pos[index]   =   ctrl_parameter['d_arm']*value
            
        self.rotor_dir  =   ctrl_parameter['rotor_directions']
        
        # rotor properties
        self.k_eta  =   ctrl_parameter['k_eta']
        self.k_m    =   ctrl_parameter['k_m']
        self.c_m    =   self.k_m/self.k_eta             # [m] ratio of yaw torque to thrust

        # motor properties
        self.rotor_speed_min    =   ctrl_parameter['rotor_speed_min']
        self.rotor_speed_max    =   ctrl_parameter['rotor_speed_max']

        # compute the control effectiveness matrix (rotor force to total F and M in body frame)
        self.CEM    =   np.empty((3+1,ctrl_parameter['num_rotors']), dtype=np.float64)

        self.CEM[0,:]   =   -np.ones((1,ctrl_parameter['num_rotors']), dtype=np.float64)
        for rotor_idx in range(0,ctrl_parameter['num_rotors']):
            self.CEM[1:3,rotor_idx] =   np.cross(self.rotor_pos[rotor_idx],np.array([0.0, 0.0, 1.0]))[0:2]

        self.CEM[3,:]   =   self.c_m*ctrl_parameter['rotor_directions']
        self.CEMinv     =   np.linalg.inv(self.CEM)

        # Gains  
        self.kpos   =   ctrl_parameter['kp']
        self.kvel   =   ctrl_parameter['kv']
        self.katt   =   ctrl_parameter['kr']
        self.krat   =   ctrl_parameter['kw']

    # get_rot_mat
    # compute the rotation matrix from the quaternion
    # input: quaternion, attitude of the quadrotor
    # output: rotation matrix, from inertial to body
    # to be updated: check the sign
    def get_rot_mat(self, q):
        rot_mat         =   np.empty((3,3), dtype=np.float64)
        rot_mat[0,0]    =   q[0]*q[0]+q[1]*q[1]-q[2]*q[2]-q[3]*q[3]
        rot_mat[0,1]    =   2*(q[1]*q[2]+q[0]*q[3])
        rot_mat[0,2]    =   2*(q[1]*q[3]-q[0]*q[2])
        rot_mat[1,0]    =   2*(q[1]*q[2]-q[0]*q[3])
        rot_mat[1,1]    =   q[0]*q[0]+q[2]*q[2]-q[1]*q[1]-q[3]*q[3]
        rot_mat[1,2]    =   2*(q[2]*q[3]+q[0]*q[1])
        rot_mat[2,0]    =   2*(q[1]*q[3]+q[0]*q[2])
        rot_mat[2,1]    =   2*(q[2]*q[3]-q[0]*q[1])
        rot_mat[2,2]    =   q[0]*q[0]+q[3]*q[3]-q[1]*q[1]-q[2]*q[2]

        return rot_mat

    def normalize(self, vec):
        """Return normalized vector."""
        return vec/np.linalg.norm(vec)

    def vee_map(self, mat):
        """Return vector corresponding to given skew symmetric matrix."""
        return np.array([0.5*(-mat[1,2]+mat[2,1]), 0.5*(mat[0,2]-mat[2,0]), 0.5*(-mat[0,1]+mat[1,0])])


    def update(self, t, state, desired_flat):

        u_cmd_rot_speeds    =   np.zeros((4,), dtype=np.float64)

        # get the desired force vector.
        pos_err     =   state['x']-desired_flat['x']
        vel_err     =   state['v']-desired_flat['xdot']

        f_des       =   -self.kpos*pos_err-self.kvel*vel_err+self.mass*(desired_flat['xddot']-np.array([0, 0, const['g']]))

        # rot
        R           =   self.get_rot_mat(state['q']).T
        b3          =   R@np.array([0, 0, 1], dtype=np.float64)
        u1          =   -np.dot(f_des, b3)

        # desired orientation to obtain force vector.
        b3_des      =   -self.normalize(f_des)
        yaw_des     =   desired_flat['yaw']
        b1_des      =   np.array([np.cos(yaw_des), np.sin(yaw_des), 0])
        b2_des      =   self.normalize(np.cross(b3_des, b1_des))
        R_des       =   np.stack([np.cross(b2_des, b3_des), b2_des, b3_des], axis=1)

        # orientation error.
        att_err     =   self.vee_map(0.5*(R_des.T@R-R.T@R_des))

        # print(att_err)

        #
        w_des       =   np.array([0, 0, desired_flat['yawdot']])
        w_err = state['w'] - w_des

        # Desired torque, in units N-m.
        u2 = self.J @ (-self.katt*att_err - self.krat*w_err) + np.cross(state['w'], self.J@state['w'])  # Includes compensation for wxJw component

        # Compute motor speeds. Avoid taking square root of negative numbers.
        TM = np.array([u1, u2[0], u2[1], u2[2]])
        u_cmd_rot_thrust = self.CEMinv@TM
        u_cmd_rot_speeds = u_cmd_rot_thrust/self.k_eta
        u_cmd_rot_speeds = np.sqrt(np.abs(u_cmd_rot_speeds))

        # print(u_cmd_rot_speeds)

        return u_cmd_rot_speeds