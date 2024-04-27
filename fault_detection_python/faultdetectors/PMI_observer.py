# import public modules
import numpy as np
from scipy import integrate

# import private modules
from faultdetectors.load_parameters import get_observer_parameters, get_miscellaneous_constants

# get constants
const   =   get_miscellaneous_constants()

# proportional multiple-integral observer
# observer that estimates the state as well as the unknown input vector to the system
# state vector xhat and unknown input vector uf (with derivatives)
# state vector: [Euler angle and body rate] (for small angle approximate (phidot, thetadot, psidot) = (p, q, r))
# euler angle and rate (phi, phidot, theta, thetadot, psi, psidot)
# to be updated:
# 1. integrate with matlab script computing the gain
# 2. check the formulation with body rate and euler angle
class PMI_observer():
    
    # __init__
    # PMI observer class initialization
    # input: model, quadrotor model
    # input: initial_estimate, initial state vector of the estimator
    # output: observer class instance with required parameters/models
    def __init__(self, model = "parrotmambo", \
                 initial_state = np.zeros((15,), dtype=np.float64)):
        
        obsv_parameter = get_observer_parameters(model)
        
        # assign parameters from the dictionary
        # inertial parameters
        self.mass   =   obsv_parameter['mass']          # [kg] mass
        self.Ixx    =   obsv_parameter['Ixx']           # [kg*m^2] x moment of inertia 
        self.Iyy    =   obsv_parameter['Iyy']           # [kg*m^2] y moment of inertia 
        self.Izz    =   obsv_parameter['Izz']           # [kg*m^2] z moment of inertia 
        
        # geometric parameters
        self.rotor_pos  =   np.empty((obsv_parameter['num_rotors'],3), dtype=np.float64)

        for index, (key, value) in enumerate(obsv_parameter['rotor_config'].items()):
            self.rotor_pos[index]   =   obsv_parameter['d_arm']*value
            
        self.rotor_dir  =   obsv_parameter['rotor_directions']

        # rotor properties
        self.k_eta  =   obsv_parameter['k_eta']
        self.k_m    =   obsv_parameter['k_m']
        self.c_m    =   self.k_m/self.k_eta             # [m] ratio of yaw torque to thrust

        # motor properties
        self.rotor_speed_min    =   obsv_parameter['rotor_speed_min']
        self.rotor_speed_max    =   obsv_parameter['rotor_speed_max']

        # compute the control effectiveness matrix (rotor force to total F and M in body frame)
        self.CEM    =   np.empty((3+1,obsv_parameter['num_rotors']), dtype=np.float64)

        self.CEM[0,:]   =   -np.ones((1,obsv_parameter['num_rotors']), dtype=np.float64)
        for rotor_idx in range(0,obsv_parameter['num_rotors']):
            self.CEM[1:3,rotor_idx] =   np.cross(self.rotor_pos[rotor_idx],np.array([0.0, 0.0, 1.0]))[0:2]

        self.CEM[3,:]   =   self.c_m*obsv_parameter['rotor_directions']

        # observer parameters
        self.xhat0 =   initial_state

        self.rho_max    =   np.array([30/180*np.pi, 30/180*np.pi], dtype=np.float64)
        self.rho_min    =   np.array([-30/180*np.pi, -30/180*np.pi], dtype=np.float64)

        c1  =   (self.Iyy-self.Izz)/self.Ixx
        c2  =   (self.Izz-self.Ixx)/self.Iyy
        c3  =   (self.Ixx-self.Iyy)/self.Izz

        self.Aset   =  np.empty((6,6,np.power(2,len(self.rho_max))), dtype=np.float64)

        self.Aset[:,:,0]    =   np.array([[0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
                                          [0.0, 0.0, 0.0, 0.0, 0.0, c1*self.rho_min[0]],
                                          [0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
                                          [0.0, 0.0, 0.0, 0.0, 0.0, c2*self.rho_min[1]],
                                          [0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
                                          [0.0, 0.0, 0.0, c3*self.rho_min[1], 0.0, 0.0]], dtype=np.float64)
        
        self.Aset[:,:,1]    =   np.array([[0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
                                          [0.0, 0.0, 0.0, 0.0, 0.0, c1*self.rho_max[0]],
                                          [0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
                                          [0.0, 0.0, 0.0, 0.0, 0.0, c2*self.rho_min[1]],
                                          [0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
                                          [0.0, 0.0, 0.0, c3*self.rho_min[1], 0.0, 0.0]], dtype=np.float64)

        self.Aset[:,:,2]    =   np.array([[0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
                                          [0.0, 0.0, 0.0, 0.0, 0.0, c1*self.rho_min[0]],
                                          [0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
                                          [0.0, 0.0, 0.0, 0.0, 0.0, c2*self.rho_max[1]],
                                          [0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
                                          [0.0, 0.0, 0.0, c3*self.rho_max[1], 0.0, 0.0]], dtype=np.float64)

        self.Aset[:,:,3]    =   np.array([[0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
                                          [0.0, 0.0, 0.0, 0.0, 0.0, c1*self.rho_max[0]],
                                          [0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
                                          [0.0, 0.0, 0.0, 0.0, 0.0, c2*self.rho_max[1]],
                                          [0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
                                          [0.0, 0.0, 0.0, c3*self.rho_max[1], 0.0, 0.0]], dtype=np.float64)

        self.Bu     =   np.array([[0.0, 0.0, 0.0],
                                  [1/self.Ixx, 0.0, 0.0],
                                  [0.0, 0.0, 0.0],
                                  [0.0, 1/self.Iyy, 0.0],
                                  [0.0, 0.0, 0.0],
                                  [0.0, 0.0, 1/self.Izz]], dtype=np.float64)
        
        self.C      =   np.eye(6, dtype=np.float64)
        
        # PMI observer gain (by solving LMI from MATLAB)
        self.Lobv0  =   np.array([[2.0862, 0.9628, 0.0000, 0.0000, 0.0000, 0.0009],
                                  [26.6639, 146.3678, 0.0000, 0.0000, 0.0006, 0.0413],
                                  [0.0000, 0.0000, 2.1500, 0.9835, -0.0000, 0.0012],
                                  [0.0000, 0.0000, 21.7976, 122.6224, -0.0270, 0.0366],
                                  [0.0001, -0.0000, 0.0002, -0.0004, 2.2999, 1.0543],
                                  [0.0404, 0.0259, 0.0603, 0.0238, 15.6257, 91.6418]], dtype=np.float64)

        self.Lobv1  =   np.array([[0.0075, 0.0132, 0.0000, 0.0000, 0.0000, 0.0000],
                                  [0.0000, 0.0000, 0.0078, 0.0133, 0.0000, 0.0000],
                                  [0.0000, 0.0000, 0.0000, 0.0000, 0.0084, 0.0136]], dtype=np.float64)

        self.Lobv2  =   np.array([[0.0054, 0.0248, 0.0000, 0.0000, 0.0000, 0.0000],
                                  [0.0000, 0.0000, 0.0054, 0.0246, 0.0000, 0.0000],
                                  [0.0000, 0.0000, 0.0000, 0.0000, 0.0053, 0.0243]], dtype=np.float64)

        self.Lobv3  =   np.array([[0.0033, 0.0077, 0.0000, 0.0000, 0.0000, 0.0000],
                                  [0.0000, 0.0000, 0.0033, 0.0076, 0.0000, 0.0000],
                                  [0.0000, 0.0000, 0.0000, 0.0000, 0.0034, 0.0076]], dtype=np.float64)

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

    # compute_body_moment
    # compute the moment from the rotor speed command
    # input: u, control input (rotor speed command) of a quadrotor
    # output: Mb, moment represented in the body frame
    def compute_body_moment(self, u):

        # compute the thrust moment
        Ft      =   -np.array([0, 0, self.k_eta])[:, np.newaxis]*np.power(u,2)        
        Mt      =   np.einsum('ijk,jk->i', self.hat_map(self.rotor_pos), Ft)
        Mt[2]   =   np.einsum('i,i->', -self.rotor_dir*self.k_m, np.power(u,2))

        return Mt
    
    # quaternion_to_euler
    # convert the quaternion measurement into euler angle measurement
    # input: quat, quaternion vector of a quadrotor
    # output: eul, euler angle vector
    # to be updated: change into euler angle rate measurement
    def quaternion_to_euler(self, q):

        eul     =   np.empty((3,), dtype=np.float64)
        eul[0]  =   np.arctan2(2*(q[0]*q[1]+q[2]*q[3]), q[0]*q[0]+q[3]*q[3]-q[1]*q[1]-q[2]*q[2])
        eul[1]  =   -np.pi/2+2*np.arctan2(np.sqrt(1+2*(q[0]*q[2]-q[1]*q[3])),np.sqrt(1-2*(q[0]*q[2]-q[1]*q[3])))
        eul[2]  =   np.arctan2(2*(q[0]*q[3]+q[1]*q[2]),1-2*(q[2]*q[2]+q[3]*q[3]))

        return eul

    # compute_xhatdot
    # compute the derivatives of the state estimate vector with control input
    # input: t, current time step
    # input: xhat, state estimate vector (state, fault vector)
    # input: state, state dictionary of a quadrotor
    # input: u, control input (rotor speed command) of a quadrotor
    # output: xhatdot, the derivative of the state estimate vector
    def compute_xhatdot(self, t, xhat, state, u):
        
        # get the force and moment
        Mb = self.compute_body_moment(u)

        # compute the scheduling parameter
        alpha_low   =   np.array([np.abs((self.rho_max[0]-state['w'][1])/(self.rho_max[0]-self.rho_min[0])),
                                  np.abs((self.rho_max[1]-state['w'][0])/(self.rho_max[1]-self.rho_min[1]))],
                                  dtype=np.float64)
        alpha_high  =   np.ones(alpha_low.shape, dtype=np.float64)-alpha_low

        mu          =   np.zeros((4,), dtype=np.float64)
        mu[0]       =   alpha_low[0]*alpha_low[1]
        mu[1]       =   alpha_high[0]*alpha_low[1]
        mu[2]       =   alpha_low[0]*alpha_high[1]
        mu[3]       =   alpha_high[0]*alpha_high[1]

        # compute the derivative
        xhatdot     =   np.zeros(xhat.shape, dtype=np.float64)

        eul         =   self.quaternion_to_euler(state['q'])
        y           =   np.array([eul[0], state['w'][0], eul[1], state['w'][1], eul[2], state['w'][2]], dtype=np.float64)

        for idx in range(0,len(mu)):
            xhatdot[0:6]    +=  mu[idx]*np.ravel(self.Aset[:,:,idx]@xhat[0:6][:, np.newaxis]+self.Bu@Mb[:, np.newaxis]+self.Bu@xhat[6:9][:,np.newaxis]+ \
                                                  self.Lobv0@(y[:,np.newaxis]-self.C@xhat[0:6][:, np.newaxis]))
            xhatdot[6:9]    +=  mu[idx]*np.ravel(self.Lobv1@(y[:,np.newaxis]-self.C@xhat[0:6][:, np.newaxis])+xhat[9:12][:,np.newaxis])
            xhatdot[9:12]   +=  mu[idx]*np.ravel(self.Lobv2@(y[:,np.newaxis]-self.C@xhat[0:6][:, np.newaxis])+xhat[12:15][:,np.newaxis])
            xhatdot[12:15]  +=  mu[idx]*np.ravel(self.Lobv3@(y[:,np.newaxis]-self.C@xhat[0:6][:, np.newaxis]))

        return xhatdot
    
    # step
    # integrate the continuous-time estimator forward for tstep from the state estimate vector/control input/state measurement
    # input: ti, current time step
    # input: statehati, state dictionary of the estimator
    # input: u, control input (rotor speed command) of a quadrotor
    # input: dt, time step size
    # output: tf, the propagated time step
    # output: statehatf, the propagated state dictionary
    def step(self, ti, xhati, statei, u, dt):
        
        # # implement the integration (Euler)
        sf      =   xhati+self.compute_xhatdot(ti, xhati, statei, u)*dt

        # # update the time
        tf      =   ti+dt
        
        return (tf, sf)