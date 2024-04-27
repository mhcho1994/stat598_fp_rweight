# import public modules
import numpy as np
from scipy import integrate

# import privatve modules
from vehicles.load_parameters import get_multirotor_parameters, get_miscellaneous_constants

# get constants
const   =   get_miscellaneous_constants()

# multirotor dynamics model
# state vector: [position, velocity, attitude, body rates]
# position (x, y, z), velocity (vx, vy, vz)
# attitude (q0, q1, q2, q3), angular rate (p, q, r)
# to be updated:
# 1. include the rotor dynamics (1st order system, rotor rotation speed as the input)
# 2. include the aerodynamic drag (the current version only considers the gyroscopic effect)
# 3. include the effect of wind (as disturbance model)
class multirotor():

    # __init__
    # multirotor class initialization
    # input: model, quadrotor model
    # input: initial_state, initial state of the quadrotor
    # output: quadrotor class instance with parameters
    def __init__(self, model = "parrotmambo", \
                 initial_state = {'x': np.zeros((3,), dtype=np.float64),
                                  'v': np.zeros((3,), dtype=np.float64),
                                  'q': np.array([1, 0, 0, 0], dtype=np.float64),
                                  'w': np.zeros((3,), dtype=np.float64)}):
        
        quad_parameter  =   get_multirotor_parameters(model)

        # assign parameters from the dictionary
        # inertial parameters
        self.mass   =   quad_parameter['mass']                                                              # [kg] mass
        self.J      =   np.array([[quad_parameter['Ixx'], quad_parameter['Ixy'], quad_parameter['Ixz']],
                                  [quad_parameter['Ixy'], quad_parameter['Iyy'], quad_parameter['Iyz']],
                                  [quad_parameter['Ixz'], quad_parameter['Iyz'], quad_parameter['Izz']]])   # [kg*m^2] inertia
        self.Jinv   =   np.linalg.inv(self.J)       # [1/kg/m^2] inverse of inertia
        self.Ir     =   quad_parameter['Ir']        # [kg*m^2] rotor inertia

        # geometric parameters
        self.rotor_pos  =   np.empty((quad_parameter['num_rotors'],3), dtype=np.float64)

        for index, (key, value) in enumerate(quad_parameter['rotor_config'].items()):
            self.rotor_pos[index]   =   quad_parameter['d_arm']*value
            
        self.rotor_dir  =   quad_parameter['rotor_directions']
        
        # aerodynamic properties
        self.drag_coeff =   np.array([quad_parameter['c_Dx'], quad_parameter['c_Dy'], quad_parameter['c_Dz']])
        
        # rotor properties
        self.k_eta  =   quad_parameter['k_eta']
        self.k_m    =   quad_parameter['k_m']
        self.c_m    =   self.k_m/self.k_eta             # [m] ratio of yaw torque to thrust

        # motor properties
        self.rotor_speed_min    =   quad_parameter['rotor_speed_min']
        self.rotor_speed_max    =   quad_parameter['rotor_speed_max']

        # set initial state in case if the state information is solely from 
        self.initial_state  =   initial_state

        # compute the control effectiveness matrix (rotor force to total F and M in body frame)
        self.CEM    =   np.empty((3+1,quad_parameter['num_rotors']), dtype=np.float64)

        self.CEM[0,:]   =   -np.ones((1,quad_parameter['num_rotors']), dtype=np.float64)
        for rotor_idx in range(0,quad_parameter['num_rotors']):
            self.CEM[1:3,rotor_idx] =   np.cross(self.rotor_pos[rotor_idx],np.array([0.0, 0.0, 1.0]))[0:2]

        self.CEM[3,:]   =   self.c_m*quad_parameter['rotor_directions']
        self.CEMinv     =   np.linalg.inv(self.CEM)

    # vectorize
    # vectorize the state vector
    # input: state, dictionary of position/velocity/quaternion/rate
    # output: s, state vector
    def vectorize(self, state):

        s   =   np.zeros((13,), dtype=np.float64)
        s[0:3]      =   state['x']
        s[3:6]      =   state['v']
        s[6:10]     =   state['q']
        s[10:13]    =   state['w']

        return s

    # decompose
    # decompose the state vector to state dictionary
    # input: s, state vector of the quadrotor
    # output: state, dictionary of position/velocity/quaternion/rate
    def decompose(self, s):

        state   =   {'x': s[0:3], 'v': s[3:6], 'q': s[6:10], 'w': s[10:13]}

        return state
    
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
    
    # hat_map
    # compute the skew-symmetric matrix for cross product
    # input: vector or matrix, in R^3 or R^(Nx3)
    # output: skew symmetric matrix (hat map), in R^(3x3) or R^(3X3XN)
    def hat_map(self, vec):

        if len(vec.shape) > 1:
            return np.array([[np.zeros(vec.shape[0]), -vec[:,2], vec[:,1]],
                             [vec[:,2], np.zeros(vec.shape[0]), -vec[:,0]],
                             [-vec[:,1], vec[:,0], np.zeros(vec.shape[0])]], dtype=np.float64)
        
        else:
            return np.array([[0.0, -vec[2], vec[1]],
                             [vec[2], 0.0, -vec[0]],
                             [-vec[1], vec[0], 0.0]], dtype=np.float64)
        
    # compute_body_wrench
    # compute the force and moment in the body frame
    # input: s, state vector of the quadrotor
    # input: u, control input (rotor speed command) of a quadrotor
    # output: (Fb, Mb), force and moment represented in the body frame
    def compute_body_wrench(self, s, u):
        
        # compute the thrust force/moment
        Ft      =   -np.array([0, 0, self.k_eta])[:, np.newaxis]*np.power(u,2)          
        Mt      =   np.einsum('ijk,jk->i', self.hat_map(self.rotor_pos), Ft)
        Mt[2]   =   np.einsum('i,i->', -self.rotor_dir*self.k_m, np.power(u,2))
        Ft      =   np.sum(Ft, axis=1)
        
        # compute the aerodynamic force/moment
        Fa      =   np.zeros((3,), dtype=np.float64)
        Ma      =   np.zeros((3,), dtype=np.float64)

        # total force/moment
        Fb      =   Ft+Fa
        Mb      =   Mt+Ma

        return (Fb, Mb)
    
    # compute_quat_dot
    # compute the derivative of quaternion using the body rate
    # input: q, quaternion (inertial -> body)
    # input: w, body rate (p, q, r)
    # output: qdot, quaternion derivative
    def compute_quat_dot(self, q, w):
        
        k       =   2
        epsilon =   1-np.sum(np.power(q,2))
        qdot    =   0.5*np.array([[0.0, -w[0], -w[1], -w[2]],
                                 [w[0], 0.0, w[2], -w[1]],
                                 [w[1], -w[2], 0.0, w[0]],
                                 [w[2], w[1], -w[0], 0.0]])@q+k*epsilon*q
        
        return qdot
    
    # compute_sdot
    # compute the derivatives of the state vector with control input
    # input: t, current time step
    # input: s, state vector of a quadrotor
    # input: u, control input (rotor speed command) of a quadrotor
    # output: sdot, the derivative of the state vector
    def compute_sdot(self, t, s, u):
        
        # get the force and moment
        (Fb, Mb) = self.compute_body_wrench(s, u)

        # position derivative (inertial velocity)
        xdot    =   s[3:6]

        # velocity derivative
        R       =   self.get_rot_mat(s[6:10])
        vdot    =   (self.mass*const['g']*np.array([0.0,0.0,1.0], dtype=np.float64)+R.T@Fb)/self.mass

        # quaternion derivative
        qdot    =   self.compute_quat_dot(s[6:10],s[10:13])

        # rate derivative
        what    =   self.hat_map(s[10:13])
        wdot    =   self.Jinv@(Mb-what@(self.J@s[10:13]))

        sdot    =   np.concatenate((xdot,vdot,qdot,wdot), axis=None)
        
        return sdot

    # step
    # integrate the dynamics forward for tstep from the state and control input
    # input: ti, current time step
    # input: statei, state dictionary of a quadrotor
    # input: u, control input (rotor speed command) of a quadrotor
    # input: dt, time step size
    # output: tf, the propagated time step
    # output: statef, the propagated state dictionary
    def step(self, ti, statei, u, dt):
        
        # vectorize the input state dictionary
        si      =   self.vectorize(statei)
        
        # implement the integration (RK45)
        sol     =   integrate.solve_ivp(fun = lambda t, s : self.compute_sdot(t, s, u), t_span = [ti, ti+dt], y0 = si, method = 'RK45', first_step = dt)
        sf      =   sol.y[:,-1]

        # decompose the state vector into a dictionary
        statef  =   self.decompose(sf)
        
        # normalize the quaternion
        statef['q'] =   statef['q']/np.linalg.norm(statef['q'])
        
        # update the time
        tf      =   sol.t[:-1]
        
        return (tf, statef)




