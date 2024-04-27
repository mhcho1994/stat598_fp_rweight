# import public modules
import numpy as np

# function: get_observer_parameters(model)
# description: 
# input: model,
# output: 
def get_observer_parameters(model):

    if (model == "parrotmambo"):
        quad_params     =   {

            # inertial properties
            'mass': np.float64(0.063),      # [kg] mass of the quadrotor
            'Ixx': np.float64(5.829e-5),    # [kg*m^2] x moment of inertia 
            'Iyy': np.float64(7.169e-5),    # [kg*m^2] y moment of inertia
            'Izz': np.float64(1e-4),        # [kg*m^2] z moment of inertia
            'Ixy': np.float64(0.0),         # [kg*m^2] xy product inertia
            'Iyz': np.float64(0.0),         # [kg*m^2] yz product inertia
            'Ixz': np.float64(0.0),         # [kg*m^2] xz product inertia

            # geometric configuration
            'num_rotors': np.uint8(4),              # [-] number of rotors
            'rotor_radius': np.float64(0.00),       # [m] rotor radius
            'd_arm': np.float64(0.0624*np.sqrt(2)), # [m] arm length (com to rotor)
            'rotor_config': {  
                'r1': np.array([np.cos(45/180*np.pi), np.sin(45/180*np.pi), 0.0]),      # [-] direction of rotor 1
                'r2': np.array([np.cos(135/180*np.pi), np.sin(135/180*np.pi), 0.0]),    # [-] direction of rotor 1
                'r3': np.array([np.cos(225/180*np.pi), np.sin(225/180*np.pi), 0.0]),    # [-] direction of rotor 1
                'r4': np.array([np.cos(315/180*np.pi), np.sin(315/180*np.pi), 0.0]),    # [-] direction of rotor 1
                },
            'rotor_directions': np.array([1,-1,1,-1], dtype=np.int8),                   # [-] rotor rotation direction (+1: CW, -1: CCW) (CW rotation -> CCW torque to body/-z direction, with -z direction force)

            # rotor properties
            'k_eta': np.float64(4.720e-6),  # [N/(rad/s)^2] thrust coefficient 
            'k_m': np.float64(1.139e-7),    # [N*m/(rad/s)^2] yaw moment coefficient 

            # motor properties
            'rotor_speed_min': np.float64(0.0),     # [rpm] minimum rotor speed
            'rotor_speed_max': np.float64(2250),    # [rpm] maximum rotor speed
            
        }

    return quad_params

# function: get_miscellaneous_constants()
# description: get the constants for a simulation
# output: const, dictionary that contains the constants
#         rad2deg, deg2rad, rpm2radps, radps2rpm, g
def get_miscellaneous_constants():
    const     =   {

        'rad2deg': np.float64(180/np.pi),       # [-] radian to degree
        'deg2rad': np.float64(np.pi/180),       # [-] degree to radian
        'rpm2radps': np.float64(2*np.pi/60),    # [-] rpm to radian per second
        'radps2rpm': np.float64(60/2/np.pi),    # [-] rpm to radian per second
        'g': np.float64(9.81),     # [kg*m^2] rotor inertia
    }

    return const