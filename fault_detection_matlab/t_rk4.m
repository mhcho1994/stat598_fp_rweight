function [Attitude, Altitude, Position, Quadrotor, Fault, Control, Residual] = t_rk4(t_dynamics, tspan, initial, F, h, UAV, R, choice, motor_f, Reference)

xr0_attitude           =  initial.xa_r0;       % [-] Attitude Reference
x0_attitude            =  initial.xa_0;        % [-] Attitude 
xhat0_attitude         =  initial.xa_hat;      % [-] Attitude estimates 

xr0_altitude           =  initial.xz_r0;       % [-] Altitude Reference
x0_altitude            =  initial.xz_0;        % [-] Altitude

xr0_position           =  initial.xp_r0;       % [-] Position Reference
x0_position            =  initial.xp_0;        % [-] Position

f0_fault_234           =  F.fault_234;         % [-] fault 234 (real)
f0_fault_234_hat       =  F.fault_234_hat;     % [-] fault 234 estimates
f0_fault_234_first     =  F.fault_234_first;   % [-] fault 234 first 
f0_fault_234_second    =  F.fault_234_first;   % [-] fault 234 second
    
nSize                  =  ceil((tspan(2)-tspan(1))/h);
nStates                =  size(x0_attitude,1);
T                      =  zeros(nSize,1);

X_Attitude             =  zeros(nSize, nStates);
XR_Attitude            =  zeros(nSize, nStates);
Xhat_Attitude          =  zeros(nSize, nStates);

X_Altitude             =  zeros(nSize, 2);
XR_Altitude            =  zeros(nSize, 2);

X_Position             =  zeros(nSize, 4);
XR_Position            =  zeros(nSize, 4);

x1                     =  x0_attitude;            % [-] Attitude
x2                     =  xr0_attitude;           % [-] Attitude Reference
x3                     =  x0_altitude;            % [-] Altitude
x4                     =  xr0_altitude;           % [-] Altitude Reference
x5                     =  x0_position;            % [-] Position
x6                     =  xr0_position;           % [-] Position Reference
x7                     =  xhat0_attitude;         % [-] Attitude Estimates 
x8                     =  f0_fault_234;           % [-] Fault 234
x9                     =  f0_fault_234_hat;       % [-] Fault 234 Estimates
x10                    =  f0_fault_234_first;     % [-] Fault 234 Estimates First
x11                    =  f0_fault_234_second;    % [-] Fault 234 Estimates Second

count                  =  1;

% [-] f_tmp           : faule_234 (real)
% [-] fhat_tmp        : falue_234 estimates
% [-] fhat_first_tmp  : fault_234 first estimates
% [-] fhat_second_tmp : fault_234 second estimates

for t = tspan(1):h:tspan(2)
    
    % Runge–Kutta step 1
    [x1_tmp, x1_rtmp, ref_attitude, x2_tmp, x2_rtmp, ref_altitude, x3_tmp, x3_rtmp, ref_position, x1hat_tmp, f_tmp, fhat_tmp, fhat_first_tmp, fhat_second_tmp, u1, u2, u3, u4, omega, r1] = feval(t_dynamics, t, x1, x2, x3, x4, x5, x6,  x7,  x8, x9, x10, x11, UAV, R, choice, motor_f, Reference);
    
    k1_attitude     =  h*x1_tmp;
    k1_attitude_r   =  h*x1_rtmp;
    k1_attitude_hat =  h*x1hat_tmp;

    k1_altitude     =  h*x2_tmp;
    k1_altitude_r   =  h*x2_rtmp;
    
    k1_position     =  h*x3_tmp;
    k1_position_r   =  h*x3_rtmp;    

    k1_f_234        =  h*fhat_tmp;
    k1_f_234_1st    =  h*fhat_first_tmp;
    k1_f_234_2st    =  h*fhat_second_tmp;

    % Runge–Kutta step 2
    [x1_tmp, x1_rtmp, ~, x2_tmp, x2_rtmp, ~, x3_tmp, x3_rtmp, ~, x1hat_tmp, ~, fhat_tmp, fhat_first_tmp, fhat_second_tmp] = feval(t_dynamics, t+h/2, x1+k1_attitude/2, x2+k1_attitude_r/2, x3+k1_altitude/2, x4+k1_altitude_r/2, x5+k1_position/2, x6+k1_position_r/2, x7+k1_attitude_hat/2, x8, ...
                                                                                                                                  x9+k1_f_234/2, x10+k1_f_234_1st/2, x11+k1_f_234_2st/2, UAV, R, choice, motor_f, Reference);
    k2_attitude     =  h*x1_tmp;
    k2_attitude_r   =  h*x1_rtmp;
    k2_attitude_hat =  h*x1hat_tmp;

    k2_altitude     =  h*x2_tmp;
    k2_altitude_r   =  h*x2_rtmp;
    
    k2_position     =  h*x3_tmp;
    k2_position_r   =  h*x3_rtmp;    
    
    k2_f_234        =  h*fhat_tmp;
    k2_f_234_1st    =  h*fhat_first_tmp;
    k2_f_234_2st    =  h*fhat_second_tmp;

    % Runge–Kutta step 3
    [x1_tmp, x1_rtmp, ~, x2_tmp, x2_rtmp, ~, x3_tmp, x3_rtmp, ~, x1hat_tmp, ~, fhat_tmp, fhat_first_tmp, fhat_second_tmp] = feval(t_dynamics, t+h/2, x1+k2_attitude/2, x2+k2_attitude_r/2, x3+k2_altitude/2, x4+k2_altitude_r/2, x5+k2_position/2, x6+k2_position_r/2, x7+k2_attitude_hat/2, x8, ...
                                                                                                                                  x9+k2_f_234/2, x10+k2_f_234_1st/2, x11+k2_f_234_2st/2, UAV, R, choice, motor_f, Reference);
    
    k3_attitude     =  h*x1_tmp;
    k3_attitude_r   =  h*x1_rtmp;
    k3_attitude_hat =  h*x1hat_tmp;

    k3_altitude     =  h*x2_tmp;
    k3_altitude_r   =  h*x2_rtmp;
    
    k3_position     =  h*x3_tmp;
    k3_position_r   =  h*x3_rtmp;  
    
    k3_f_234        =  h*fhat_tmp;
    k3_f_234_1st    =  h*fhat_first_tmp;
    k3_f_234_2st    =  h*fhat_second_tmp;

    % Runge–Kutta step 4
    [x1_tmp, x1_rtmp, ~, x2_tmp, x2_rtmp, ~, x3_tmp, x3_rtmp, ~, x1hat_tmp, ~, fhat_tmp, fhat_first_tmp, fhat_second_tmp] = feval(t_dynamics, t+h, x1+k3_attitude, x2+k3_attitude_r, x3+k3_altitude, x4+k3_altitude_r, x5+k3_position, x6+k3_position_r, x7+k3_attitude_hat, x8, ...
                                                                                                                                  x9+k3_f_234, x10+k3_f_234_1st, x11+k3_f_234_2st, UAV, R, choice, motor_f, Reference);
    
    k4_attitude     =  h*x1_tmp;
    k4_attitude_r   =  h*x1_rtmp;
    k4_attitude_hat =  h*x1hat_tmp;

    k4_altitude     =  h*x2_tmp;
    k4_altitude_r   =  h*x2_rtmp;
    
    k4_position     =  h*x3_tmp;
    k4_position_r   =  h*x3_rtmp;  

    k4_f_234        =  h*fhat_tmp;
    k4_f_234_1st    =  h*fhat_first_tmp;
    k4_f_234_2st    =  h*fhat_second_tmp;

    xn_attitude     =  x1 + k1_attitude/6 + k2_attitude/3 + k3_attitude/3 + k4_attitude/6;
    xrn_attitude    =  x2 + k1_attitude_r/6 + k2_attitude_r/3 + k3_attitude_r/3 + k4_attitude_r/6;
    xn_altitude     =  x3 + k1_altitude/6 + k2_altitude/3 + k3_altitude/3 + k4_altitude/6;
    xrn_altitude    =  x4 + k1_altitude_r/6 + k2_altitude_r/3 + k3_altitude_r/3 + k4_altitude_r/6;
    xn_position     =  x5 + k1_position/6 + k2_position/3 + k3_position/3 + k4_position/6;
    xrn_position    =  x6 + k1_position_r/6 + k2_position_r/3 + k3_position_r/3 + k4_position_r/6;
    xhat_attitude   =  x7 + k1_attitude_hat/6 + k2_attitude_hat/3 + k3_attitude_hat/3 + k4_attitude_hat/6;

    f_234_hat         =  x9 + k1_f_234/6 + k2_f_234/3 + k3_f_234/3 + k4_f_234/6;
    f_234_hat_first   =  x10 + k1_f_234_1st/6 + k2_f_234_1st/3 + k3_f_234_1st/3 + k4_f_234_1st/6;
    f_234_hat_second  =  x11 + k1_f_234_2st/6 + k2_f_234_2st/3 + k3_f_234_2st/3 + k4_f_234_2st/6;

    T(count) = t;

    X_Attitude(count,:)         = x1';
    XR_Attitude(count,:)        = x2';

    X_Altitude(count,:)         = x3'; 
    XR_Altitude(count,:)        = x4'; 

    X_Position(count,:)         = x5'; 
    XR_Position(count,:)        = x6'; 

    Xhat_Attitude(count,:)      = x7';

    Fault_234(count,:)          = x8';
    Fault_234_hat(count,:)      = x9';
    Fault_234_first(count,:)    = x10';
    Fault_234_second(count,:)   = x11';

    U1_control(count,:)         = u1;
    U2_control(count,:)         = u2;
    U3_control(count,:)         = u3;
    U4_control(count,:)         = u4;

    Residual_attitude(count,:)  = r1;

    x1             = xn_attitude;
    x2             = xrn_attitude;
    x3             = xn_altitude;
    x4             = xrn_altitude;
    x5             = xn_position;
    x6             = xrn_position;
    x7             = xhat_attitude;
    x8             = f_tmp;
    x9             = f_234_hat;
    x10            = f_234_hat_first;
    x11            = f_234_hat_second;
        
    Roll(count,:)  = ref_attitude(1);     % [-] Roll
    Pitch(count,:) = ref_attitude(2);     % [-] Pitch
    Yaw(count,:)   = ref_attitude(3);     % [-] Yaw
    Z(count,:)     = ref_altitude;        % [-] Z
    X(count,:)     = ref_position(1);     % [-] X
    Y(count,:)     = ref_position(2);     % [-] Y

    %% This part is for draw UAV in the real simulation
    roll       = x1(1);
    pitch      = x1(3);
    yaw        = x1(5);
    x_position = x5(1);
    y_position = x5(2);
    z_position = x3(1);

    % Rotational Matrix
    Rotational = [cos(yaw)*cos(pitch)  -sin(yaw)*cos(roll)+cos(yaw)*sin(pitch)*sin(roll)    sin(yaw)*sin(roll)+cos(yaw)*sin(pitch)*cos(roll);
                  sin(yaw)*cos(pitch)   cos(yaw)*cos(roll)+sin(yaw)*sin(pitch)*sin(roll)   -cos(yaw)*sin(roll)+sin(yaw)*sin(pitch)*cos(roll);
                          -sin(pitch)                               cos(pitch)*sin(roll)                               cos(pitch)*cos(roll)];

    % drone1(:,count) = Rotational*[1 -1 0]'  + [x_position y_position z_position]';
    % drone2(:,count) = Rotational*[-1 -1 0]' + [x_position y_position z_position]';
    % drone3(:,count) = Rotational*[-1 1 0]'  + [x_position y_position z_position]';
    % drone4(:,count) = Rotational*[1 1 0]'   + [x_position y_position z_position]';

    drone1(:,count) = Rotational*[1 0 0]'  + [x_position y_position z_position]';
    drone2(:,count) = Rotational*[0 -1 0]' + [x_position y_position z_position]';
    drone3(:,count) = Rotational*[-1 0 0]' + [x_position y_position z_position]';
    drone4(:,count) = Rotational*[0 1 0]'  + [x_position y_position z_position]';

    count           = count + 1;
end

%% Return Values 
Attitude{1} = X_Attitude;
Attitude{2} = XR_Attitude;
Attitude{3} = Roll;
Attitude{4} = Pitch;
Attitude{5} = Yaw;
Attitude{6} = Xhat_Attitude;

Altitude{1} = X_Altitude;
Altitude{2} = XR_Altitude;
Altitude{3} = Z;

Position{1} = X_Position;
Position{2} = XR_Position;
Position{3} = X;
Position{4} = Y;

Quadrotor{1} = drone1;
Quadrotor{2} = drone2;
Quadrotor{3} = drone3;
Quadrotor{4} = drone4;

Fault{1}     =  Fault_234;           % [-] real faults 
Fault{2}     =  Fault_234_hat;       % [-] estimated faults
Fault{3}     =  Fault_234_first;     % [-] estimated first derivative
Fault{4}     =  Fault_234_second;    % [-] estimated second derivative

Control{1}   =  U1_control;
Control{2}   =  U2_control;
Control{3}   =  U3_control;
Control{4}   =  U4_control;

Residual{1}  =  Residual_attitude;

end