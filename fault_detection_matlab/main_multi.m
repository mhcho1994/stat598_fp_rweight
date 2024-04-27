clear all 
close all
clc

%% Quadrotor UAV Attitude Simulations

% mode = 1 : Trajectory Plots
% mode = 2 : UAV Videos

% choice = 0 : Nominal
% choice = 1 : Actuator Fault

mode     =  1;
choice   =  1;

%% Quadrotor UAV Parameters
m        =  0.063;          % [kg] mass of the quadrotor
g        =  9.81;           % [m/s^2] gravitational acceleration
Ix       =  0.00005829;     % [kgm^2] moment of inertia
Iy       =  0.00007169;     % [kgm^2] moment of inertia
Iz       =  0.0001;         % [kgm^2] moment of inertia
ct       =  4.72*10e-8;     % Thrust coefficient
ctau     =  1.139*10e-10;   % Drag coefficient
l        =  0.0624;         % [m]
d        =  1;              % [m]
ra       =  4;

UAV.m    =  m;       
UAV.g    =  g;       
UAV.Ix   =  Ix;   
UAV.Iy   =  Iy;   
UAV.Iz   =  Iz;   
UAV.l    =  l;    
UAV.d    =  d;       
UAV.ra   =  ra;
UAV.ct   =  ct;
UAV.ctau =  ctau;

%% Quadrotor UAV System Matrices

% ====================== Attitude Model (자세 모델) ======================

% xa(1) = Roll
% xa(2) = Roll_Dot
% xa(3) = Pitch
% xa(4) = Pitch_Dot
% xa(5) = Yaw
% xa(6) = Yaw_Dot

a1 = (UAV.Iy-UAV.Iz)/UAV.Ix;
a2 = (UAV.Iz-UAV.Ix)/UAV.Iy;
a3 = (UAV.Ix-UAV.Iy)/UAV.Iz;

b1 = UAV.l/UAV.Ix;
b2 = UAV.l/UAV.Iy;
b3 = UAV.d/UAV.Iz;

rho1_max = 0.1;
rho1_min = -0.1;
rho2_max = 0.1;
rho2_min = -0.1;

Aa{1} = [0 1 0            0    0  0;
         0 0 0            0    0  a1*rho1_min;
         0 0 0            1    0  0;
         0 0 0            0    0  a2*rho2_min;
         0 0 0            0    0  1;
         0 0 0  a3*rho2_min    0  0];

Aa{2} = [0 1 0            0    0  0;
         0 0 0            0    0  a1*rho1_max;
         0 0 0            1    0  0;
         0 0 0            0    0  a2*rho2_min;
         0 0 0            0    0  1;
         0 0 0  a3*rho2_min    0  0];

Aa{3} = [0 1 0            0    0  0;
         0 0 0            0    0  a1*rho1_min;
         0 0 0            1    0  0;
         0 0 0            0    0  a2*rho2_max;
         0 0 0            0    0  1;
         0 0 0  a3*rho2_max    0  0];

Aa{4} = [0 1 0            0    0  0;
         0 0 0            0    0  a1*rho1_max;
         0 0 0            1    0  0;
         0 0 0            0    0  a2*rho2_max;
         0 0 0            0    0  1;
         0 0 0  a3*rho2_max    0  0];

Ba    = [0 0 0; 1/Ix 0 0; 0 0 0; 0 1/Iy 0; 0 0 0; 0 0 1/Iz];
Ma    = [0 0 0; -1 0 0; 0 0 0; 0 1 0; 0 0 0; 0 0 1];

n     = size(Aa{1},1);
m     = size(Ba,2);

% State and Fault Observer
bar_A{1}  =  [              Aa{1}                Ba     zeros(n,m)      zeros(n,m);
                       zeros(m,n)        zeros(m,m)       eye(m,m)      zeros(m,m);
                       zeros(m,n)        zeros(m,m)     zeros(m,m)        eye(m,m);
                       zeros(m,n)        zeros(m,m)     zeros(m,m)     zeros(m,m)];

bar_A{2}  =  [              Aa{2}                Ba     zeros(n,m)      zeros(n,m);
                        zeros(m,n)         zeros(m,m)     eye(m,m)      zeros(m,m);
                        zeros(m,n)         zeros(m,m)   zeros(m,m)        eye(m,m);
                        zeros(m,n)         zeros(m,m)   zeros(m,m)     zeros(m,m)];

bar_A{3}  =  [              Aa{3}                Ba     zeros(n,m)      zeros(n,m);
                        zeros(m,n)         zeros(m,m)     eye(m,m)      zeros(m,m);
                        zeros(m,n)         zeros(m,m)   zeros(m,m)        eye(m,m);
                        zeros(m,n)         zeros(m,m)   zeros(m,m)     zeros(m,m)];

bar_A{4}  =  [              Aa{4}                Ba     zeros(n,m)      zeros(n,m);
                        zeros(m,n)         zeros(m,m)     eye(m,m)      zeros(m,m);
                        zeros(m,n)         zeros(m,m)   zeros(m,m)        eye(m,m);
                        zeros(m,n)         zeros(m,m)   zeros(m,m)     zeros(m,m)];

bar_F     =  [                 Ma     zeros(n,m)  zeros(n,6);  
                        zeros(m,m)      eye(m,m)  zeros(m,6);
                        zeros(m,m)    zeros(m,m)  zeros(m,6);  
                        zeros(m,m)    zeros(m,m)  zeros(m,6)];

bar_C     =  [eye(n,n) zeros(n,m) zeros(n,m) zeros(n,m)];
bar_H     =  [zeros(n,m) zeros(n,m) eye(n,n)];

SYSTEM.bar_A    =  bar_A;
SYSTEM.A        =  Aa;
SYSTEM.B        =  Ba;
SYSTEM.bar_C    =  bar_C;
SYSTEM.bar_F    =  bar_F;
SYSTEM.bar_H    =  bar_H;

UAV.Aa = Aa;
UAV.Ba = Ba;
UAV.Ca = eye(6,6);
UAV.Ha = eye(6,6);

GAIN1  = Attitude_Control_LMI(UAV);         % [-] LMI for attitude controller
GAIN2  = Attitude_Estimator_LMI(SYSTEM);    % [-] LMI for attitude estimator

Ka{1}  = 10*GAIN1;
Ka{2}  = Ka{1};
Ka{3}  = Ka{1};
Ka{4}  = Ka{1};

for i = 1 : 4
    L_attitude{i} = GAIN2{i}(1:6,1:6);      % [-] State observer gain
    L1_fault{i}   = GAIN2{i}(7:9,1:6);      % [-] fault observer gain
    L2_fault{i}   = GAIN2{i}(10:12,1:6);    % [-] fault observer gain (first derivative)
    L3_fault{i}   = GAIN2{i}(13:15,1:6);    % [-] fault observer gain (second derivative)
end

UAV.Ka            =  Ka;
UAV.Ma            =  Ma;
UAV.L_attitude    =  L_attitude;
UAV.L1_fault      =  L1_fault;
UAV.L2_fault      =  L2_fault;
UAV.L3_fault      =  L3_fault;

% ====================== Altitude Model (고도 모델) ======================

% xz(1) = Z
% xz(2) = Z_Dot

Az{1}  =  [0 1; 0 0];
Az{2}  =  Az{1};
    
Al_min =  0.25/UAV.m;
Al_max =  1/UAV.m;

Bz{1}  =  [0;Al_min];
Bz{2}  =  [0;Al_max];

UAV.Az =  Az;
UAV.Bz =  Bz;

GAIN   = Altitude_Control_LMI(UAV);      

Kz{1}  =  2*GAIN;
Kz{2}  =  Kz{1};

UAV.Kz =  Kz;

% ====================== Position Model (위치 모델) ======================

% xp(1) = X
% xp(2) = Y
% xp(3) = X_Dot
% xp(4) = Y_Dot

Ap{1}  = [0 0 1 0; 0 0 0 1; 0 0 0 0; 0 0 0 0];
Ap{2}  = Ap{1};

Po_min = 0.1/UAV.m;
Po_max = 20/UAV.m;

Bp{1}  =  [0 0; 0 0; Po_min 0; 0 Po_min];
Bp{2}  =  [0 0; 0 0; Po_max 0; 0 Po_max];

UAV.Ap =  Ap;
UAV.Bp =  Bp;

GAIN   = Position_Control_LMI(UAV);      

Kp{1}  =  2*GAIN;
Kp{2}  =  Kp{1};

UAV.Kp =  Kp;

%% Reference Models
   
% Attitude Model (자세 레퍼런스 모델)

ka_p     = 10^5;      % [-] Proportional gain
ka_d     = 10^3;      % [-] Derivative gain

bar_Aar  =  [0 1 0 0 0 0; 
             0 0 0 0 0 0; 
             0 0 0 1 0 0; 
             0 0 0 0 0 0; 
             0 0 0 0 0 1; 
             0 0 0 0 0 0];

bar_Bar  =  [0 0 0;
             1 0 0; 
             0 0 0; 
             0 1 0; 
             0 0 0; 
             0 0 1];

Kar      =  [ka_p   ka_d      0      0      0      0; 
                0      0   ka_p   ka_d      0      0; 
                0      0      0      0   ka_p   ka_d];

A_a_r    =  bar_Aar-bar_Bar*Kar;
B_a_r    =  ka_p*bar_Bar;
R.A_a_r  =  A_a_r;
R.B_a_r  =  B_a_r;

% Altitude Model (고도 레퍼런스 모델)

kz_p     = 230;       % [-] Proportional gain
kz_d     = 30;        % [-] Derivative gain

bar_Azr  =  [0 1; 0 0];
bar_Bzr  =  [0;1];
Kzr      =  [kz_p kz_d];

A_z_r    =  bar_Azr-bar_Bzr*Kzr;
B_z_r    =  kz_p*bar_Bzr;
R.A_z_r  =  A_z_r;
R.B_z_r  =  B_z_r; 

% Position Model (위치 레퍼런스 모델)

kp_p     = 500;       % [-] Proportional gain
kp_d     = 30;        % [-] Derivative gain

bar_Apr  = [0 0 1 0; 
            0 0 0 1; 
            0 0 0 0; 
            0 0 0 0];

bar_Bpr  = [0 0; 
            0 0; 
            1 0; 
            0 1];

Kpr      = [kp_p      0   kp_d      0; 
               0   kp_p      0   kp_d];

A_p_r    =  bar_Apr-bar_Bpr*Kpr;
B_p_r    =  kp_p*bar_Bpr;
R.A_p_r  =  A_p_r;
R.B_p_r  =  B_p_r; 

%% Initial Values

% Attitude (자세)
initial.xa_r0  =  [0 0 0 0 0 0]';
initial.xa_0   =  [0 0 0 0 0 0]';       
initial.xa_hat =  zeros(6,1);

% Altitude (고도)
initial.xz_r0  =  [0 0]';
initial.xz_0   =  [0 0]';               

% Position (위치)
initial.xp_r0  =  [0 0 0 0]';
initial.xp_0   =  [0 0 0 0]';           

% Actuator Fault
fault_init.fault_234         =  zeros(3,1);
fault_init.fault_234_hat     =  zeros(3,1);
fault_init.fault_234_first   =  zeros(3,1);
fault_init.fault_234_second  =  zeros(3,1);

%% Simulation Initial Condition and Time Interval
tspan      =  [0 3.7];   
h          =  0.001;
t          =  transpose(tspan(1):h:tspan(2));
T.tspan    =  tspan;
T.h        =  h;
T.t        =  t;

LENGTH     =  600;   
Roll_Save  =  [];
Pitch_Save =  [];
DATA_Save  =  [];

Sample_N        =  500;         % [-] 96%
Sample_n        =  20;          % [-] 4%
Sample_NT       =  50;

Sample_tmp0     =  1:1:Sample_N;
Sample_Small1   =  randperm(Sample_N,Sample_n);    

Sample_tmp1     =  setdiff(Sample_tmp0,Sample_Small1);
Sample_Small2   =  randsample(Sample_tmp1,Sample_n);

Sample_tmp2     =  setdiff(Sample_tmp1,Sample_Small2);
Sample_Small3   =  randsample(Sample_tmp2,Sample_n);

Sample_tmp3     =  setdiff(Sample_tmp2,Sample_Small3);

Dataset_major   =  Sample_tmp3;
Dataset_minor1  =  Sample_Small1;
Dataset_minor2  =  Sample_Small2; to this end
Dataset_minor3  =  Sample_Small3;

MOTOR           =  [];

Choice          =  1;     
Selection       =  0;

% Choice    = 0 (Regular Learning)
% Choice    = 1 (Imbalance Learning)
% Selection = 0 (Training)
% Selection = 1 (Testing)

%% Regular Learning 
if Choice == 0
    %% Training set
    if Selection == 0
        for i = 1 : 1 : Sample_N
            
            Reference = 2*rand(1);
            motor_f   = randi(4,1);        
            MOTOR     = [MOTOR, motor_f];
            
            [~, ~, ~, ~, ~, ~, Residual] = t_rk4(@Dynamics, tspan, initial, fault_init, h, UAV, R, choice, motor_f, Reference); 
            
            for j = 1 : 2
                Roll_tmp    =  Residual{1}( (j-1)*LENGTH+2500:j*LENGTH+2500-1,1 );
                Pitch_tmp   =  Residual{1}( (j-1)*LENGTH+2500:j*LENGTH+2500-1,3 );
        
                Roll_Save   =  [Roll_Save;Roll_tmp];
                Pitch_Save  =  [Pitch_Save;Pitch_tmp];
            end
        
            DATA_Value{i}   =  [Roll_Save';Pitch_Save'];  
            DATA_Label{i}   =  motor_f; 
        
            Roll_Save       =  [];
            Pitch_Save      =  [];
        
            i
        
        end
        save Residual_Training_Choice_0 DATA_Value
        save Residual_Training_Label_Choice_0 DATA_Label
    %% Testing set   
    else
        for i = 1 : 1 : Sample_NT
            
            Reference = 2*rand(1);
            motor_f   = randi(4,1);        
            MOTOR     = [MOTOR, motor_f];
            
            [~, ~, ~, ~, ~, ~, Residual] = t_rk4(@Dynamics, tspan, initial, fault_init, h, UAV, R, choice, motor_f, Reference); 
            
            for j = 1 : 2
                Roll_tmp    =  Residual{1}( (j-1)*LENGTH+2500:j*LENGTH+2500-1,1 );
                Pitch_tmp   =  Residual{1}( (j-1)*LENGTH+2500:j*LENGTH+2500-1,3 );
        
                Roll_Save   =  [Roll_Save;Roll_tmp];
                Pitch_Save  =  [Pitch_Save;Pitch_tmp];
            end
        
            DATA_Value{i}   =  [Roll_Save';Pitch_Save'];  
            DATA_Label{i}   =  motor_f; 
        
            Roll_Save       =  [];
            Pitch_Save      =  [];
        
            i
        
        end
        save Residual_Testing_Choice_0 DATA_Value
        save Residual_Testing_Label_Choice_0 DATA_Label
    end 
%% Imbalance Learning 
else
    %% Training set
    if Selection == 0
        for i = 1 : 1 : Sample_N
            
            Reference = 2*rand(1);
            
            if ismember(i,Dataset_major)
                motor_f = 1;
            elseif ismember(i,Dataset_minor1)
                motor_f = 2;
            elseif ismember(i,Dataset_minor2)
                motor_f = 3;
            elseif ismember(i,Dataset_minor3)
                motor_f = 4;                
            else
            end
            
            MOTOR = [MOTOR, motor_f];
            
            [~, ~, ~, ~, ~, ~, Residual] = t_rk4(@Dynamics, tspan, initial, fault_init, h, UAV, R, choice, motor_f, Reference); 
            
            for j = 1 : 2
                Roll_tmp    =  Residual{1}( (j-1)*LENGTH+2500:j*LENGTH+2500-1,1 );
                Pitch_tmp   =  Residual{1}( (j-1)*LENGTH+2500:j*LENGTH+2500-1,3 );
        
                Roll_Save   =  [Roll_Save;Roll_tmp];
                Pitch_Save  =  [Pitch_Save;Pitch_tmp];
            end
        
            DATA_Value{i}   =  [Roll_Save';Pitch_Save'];  
            DATA_Label{i}   =  motor_f; 
        
            Roll_Save       =  [];
            Pitch_Save      =  [];
        
            i
        
        end
        save Residual_Training_Choice_1 DATA_Value
        save Residual_Training_Label_Choice_1 DATA_Label
    %% Testing set
    else
        for i = 1 : 1 : Sample_NT
            
            Reference = 2*rand(1);
            motor_f   = datasample([1,2,3,4],1);
            MOTOR     = [MOTOR, motor_f];
            
            [~, ~, ~, ~, ~, ~, Residual] = t_rk4(@Dynamics, tspan, initial, fault_init, h, UAV, R, choice, motor_f, Reference); 
            
            for j = 1 : 2
                Roll_tmp    =  Residual{1}( (j-1)*LENGTH+2500:j*LENGTH+2500-1,1 );
                Pitch_tmp   =  Residual{1}( (j-1)*LENGTH+2500:j*LENGTH+2500-1,3 );
        
                Roll_Save   =  [Roll_Save;Roll_tmp];
                Pitch_Save  =  [Pitch_Save;Pitch_tmp];
            end
        
            DATA_Value{i}   =  [Roll_Save';Pitch_Save'];  
            DATA_Label{i}   =  motor_f; 
        
            Roll_Save       =  [];
            Pitch_Save      =  [];
        
            i
        
        end
        save Residual_Testing_Choice_1 DATA_Value
        save Residual_Testing_Label_Choice_1 DATA_Label
    end
end
