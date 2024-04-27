import numpy as np

class HoverTraj():

    def __init__(self):
        """
        This is the constructor for the Trajectory object. A fresh trajectory
        object will be constructed before each mission.
        """

    def update(self, t):
        """
        Given the present time, return the desired flat output and derivatives.

        Inputs
            t, time, s
        Outputs
            flat_output, a dict describing the present desired flat outputs with keys
                x,        position, m
                x_dot,    velocity, m/s
                x_ddot,   acceleration, m/s**2
                x_dddot,  jerk, m/s**3
                x_ddddot, snap, m/s**4
                yaw,      yaw angle, rad
                yaw_dot,  yaw rate, rad/s
        """
        x           =   np.zeros((3,))
        xdot        =   np.zeros((3,))
        xddot       =   np.zeros((3,))
        xdddot      =   np.zeros((3,))
        xddddot     =   np.zeros((3,))
        yaw         =   0
        yawdot      =   0
        yawddot     =   0

        flat_output = {'x': x, 'xdot': xdot, 'xddot': xddot, 'x_dddot': xdddot, 'x_ddddot': xddddot,
                        'yaw': yaw, 'yawdot': yawdot, 'yawddot': yawddot}
        
        return flat_output