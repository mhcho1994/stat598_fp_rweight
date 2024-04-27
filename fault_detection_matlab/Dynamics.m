function [d_attitude, dr_attitude, ref_attitude, ...
          d_altitude, dr_altitude, ref_altitude, ...
          d_position, dr_position, ref_position, dhat_attitude, ...
          fault_234, dhat_fault, dhat_fault_1st, dhat_fault_2st, ...
          U1, U2, U3, U4, Omega, ...
          residual_attitude ] = Dynamics(t, x1, x2, x3, x4, x5, x6, x7, x8, x9, x10, x11, UAV, R, choice, motor_f, Reference)

% x1  : Attitude (자세)
% x2  : Attitude Reference
% x3  : Altitude (고도)
% x4  : Altitude Reference
% x5  : Position (위치)
% x6  : Position Reference

% x7  : Attitude Estimates

% x8  : fault234
% x9  : fault234_hat
% x10 : fault234_hat_first
% x11 : fault234_hat_second

m     =  UAV.m;
g     =  UAV.g;
l     =  UAV.l;
ct    =  UAV.ct;        % [-] Thrust coefficient
ctau  =  UAV.ctau;      % [-] Drag coefficient

%% UAV System Model (UAV 시스템 모델)

% ===============================================
% ============== Fault Modeling =================
% ===============================================

L_attitude       =  UAV.L_attitude;
L1_fault         =  UAV.L1_fault;
L2_fault         =  UAV.L2_fault;
L3_fault         =  UAV.L3_fault;

fault_234        =  x8;     % [-] u_f (u_f2, u_f3, u_f4)
fault_234_hat    =  x9;     % [-] hat{u_f}
fault_234_first  =  x10;    % [-] hat{u_f}_f1
fault_234_second =  x11;    % [-] hat{u_f}_f2

dhat_fault       =  zeros(3,1);
dhat_fault_1st   =  zeros(3,1);
dhat_fault_2st   =  zeros(3,1);

% ==========================================================
% ============== Attitude (자세) Modeling ==================
% ==========================================================

ra               =  UAV.ra;
x_attitude       =  x1;
xr_attitude      =  x2;
xhat_attitude    =  x7; 

u_roll           =  0;
u_pitch          =  0;
u_yaw            =  0;

d_attitude       =  zeros(size(x_attitude,1),1);
dhat_attitude    =  zeros(size(xhat_attitude,1),1);

for i = 1 : ra
    Aa{i} = UAV.Aa{i};
    Ba{i} = UAV.Ba;
    Ka{i} = UAV.Ka{i};
end

Ca = UAV.Ca;
Ha = UAV.Ha;

% [-] Rho1 : Pitch_Dot 
% [-] Rho2 : Roll_Dot

rho1_max  =  2000*pi/180;        % [-] max
rho1_min  =  -2000*pi/180;       % [-] min
rho2_max  =  2000*pi/180;        % [-] max
rho2_min  =  -2000*pi/180;       % [-] min

Rho1_min  =  (rho1_max-x_attitude(4))/(rho1_max-rho1_min);
Rho1_max  =  1-Rho1_min;

Rho2_min  =  (rho2_max-x_attitude(2))/(rho2_max-rho2_min);
Rho2_max  =  1-Rho2_min;

MU{1} = Rho1_min * Rho2_min;
MU{2} = Rho1_max * Rho2_min;
MU{3} = Rho1_min * Rho2_max;
MU{4} = Rho1_max * Rho2_max;

sum_attitude = 0;

for i = 1 : ra
    sum_attitude = sum_attitude + MU{i};
end

for i = 1 : ra
    h{i} = MU{i}/sum_attitude;
end

for i = 1 : 6
    w_a(i) = -0.005+0.01*rand(1);
    h_a(i) = -0.005+0.01*rand(1);
end

for i = 1 : ra
    u_roll    =  u_roll + h{i}*Ka{i}(1,:)*( xhat_attitude-xr_attitude );
    u_pitch   =  u_pitch + h{i}*Ka{i}(2,:)*( xhat_attitude-xr_attitude );
    u_yaw     =  u_yaw + h{i}*Ka{i}(3,:)*( xhat_attitude-xr_attitude );
end

y_attitude         =  Ca*x_attitude + Ha*h_a';          
yhat_attitude      =  Ca*xhat_attitude;
residual_attitude  =  yhat_attitude - y_attitude;

%residual_attitude  =  [Roll_d;Pitch_d;Yaw_d]-[x_attitude(1);x_attitude(3);x_attitude(5)]; 

% =========================================================
% ================== Altitude (고도) Modeling =============
% =========================================================

x_altitude     =  x3;
xr_altitude    =  x4;
sum_altitude   =  0;

d_altitude     =  zeros(size(x_altitude,1),1);

for i = 1 : 2
    Az{i} = UAV.Az{i};
    Bz{i} = UAV.Bz{i};
    Kz{i} = UAV.Kz{i};
end

Al_min  =  0.25/m;
Al_max  =  1/m;

AL_min  = (Al_max-(cos(x1(1))*cos(x1(3)))/UAV.m)/(Al_max-Al_min); 
AL_max  = 1-AL_min;

N{1}    = AL_min;
N{2}    = AL_max;

for i = 1 : 2
    sum_altitude = sum_altitude + N{i};
end

for i = 1 : 2
    w{i} = N{i}/sum_altitude;
end

for i = 1 : 2
    w_al(i) = -0.005+0.01*rand(1);
end

% =========================================================
% ============== Position (위치) Modeling =================
% =========================================================

x_position      =  x5;
xr_position     =  x6;
sum_position    =   0;
uz              =   0;
up              =   0;

d_position      =  zeros(size(x_position,1),1);

for i = 1 : 2
    Ap{i} = UAV.Ap{i};
    Bp{i} = UAV.Bp{i};
    Kp{i} = UAV.Kp{i};
end

Po_min  =  0.1/m;
Po_max  =  20/m;

for i = 1 : 2
    uz = uz + w{i}*Kz{i}*(x_altitude-xr_altitude);
end

ut = (m*g)/(cos(x7(1))*cos(x7(3))) + uz;

Position_min = (Po_max-ut/m)/(Po_max-Po_min);
Position_max = 1 - Position_min;

P{1}  = Position_min;
P{2}  = Position_max;

for i = 1 : 2
    sum_position = sum_position + P{i};
end

for i = 1 : 2
    b{i} = P{i}/sum_position;
end

for i = 1 : 2
    up = up + b{i}*Kp{i}*(x_position-xr_position);
end

for i = 1 : 4
    w_p(i) = -0.005+0.01*rand(1);
end

ux = up(1);
uy = up(2);

%% Reference Input (레퍼런스 입력)

% ============= Attitude ============= (자세 레퍼런스)
% The control signal of the position model generates the reference value of the attitude model

Roll_d    =  asin( sin(x7(5))*ux-cos(x7(5))*uy );
Pitch_d   =  asin( (cos(x7(5))*ux+sin(x7(5))*uy) /cos(Roll_d) );
Yaw_d     =  0;

ref_attitude = [Roll_d;Pitch_d;Yaw_d];

Aa_r = R.A_a_r;
Ba_r = R.B_a_r;

% ============= Position ============= (위치 & 고도 레퍼런스)

if (0<=t) && (t<=1)

    Px  = 0;
    Py  = 0;
    Z_r = 1;

elseif (1<t) && (t<=2)

    Px  = 0;
    Py  = 0;
    Z_r = 2;    

else

    Px  = Reference*sin(t-2);
    Py  = Reference*cos(t-2);
    Z_r = 2;

end

Az_r = R.A_z_r;
Bz_r = R.B_z_r;

ref_altitude = Z_r;
ref_position = [Px;Py];

Ap_r = R.A_p_r;
Bp_r = R.B_p_r;

%% UAV System Dynamics 

% Control Input
U1 = ut;            %  [-]  Total thrust
U2 = u_roll;        %  [-]  Roll toque
U3 = u_pitch;       %  [-]  Pitch toque
U4 = u_yaw;         %  [-]  Yaw thrust

Coefficient  = [ct ct ct ct; 0 l*ct 0 -l*ct; l*ct 0 -l*ct 0; ctau -ctau ctau -ctau];
Omega        = inv(Coefficient)*[U1;U2;U3;U4];
ESC_Control  = Coefficient*[Omega(1); Omega(2); Omega(3); Omega(4)];
ratio        = 0.15;

switch motor_f
    case 1
        ESC_Fault = Coefficient*[-ratio*Omega(1); 0; 0; 0];
    case 2
        ESC_Fault = Coefficient*[0; -ratio*Omega(2); 0; 0];
    case 3
        ESC_Fault = Coefficient*[0; 0; -ratio*Omega(3); 0];
    case 4
        ESC_Fault = Coefficient*[0; 0; 0; -ratio*Omega(4)];
end

% Reference Dynamics
dr_attitude = Aa_r*xr_attitude + Ba_r*ref_attitude; 
dr_altitude = Az_r*xr_altitude + Bz_r*ref_altitude; 
dr_position = Ap_r*xr_position + Bp_r*ref_position;

Fault_start = 3;

% Fault Dynamics (펄트 다이나믹스)
if choice == 0
    ESC_Fault = zeros(4,1);
elseif choice == 1
    if t >= Fault_start
       fault_234 = ESC_Fault(2:4);
    else
       ESC_Fault = zeros(4,1);
       fault_234 = zeros(3,1);
    end
end

% Fault Observer Dynamics (펄트 옵저버)
for i = 1 : ra
    % Fault estimator 
    dhat_fault        =   dhat_fault + h{i}*(L1_fault{i}*( y_attitude-yhat_attitude ) + fault_234_first);
    
    % Fault estimator (1st derivative)
    dhat_fault_1st    =   dhat_fault_1st + h{i}*(L2_fault{i}*( y_attitude-yhat_attitude ) + fault_234_second);
    
    % Fault estimator (2st derivative)
    dhat_fault_2st    =   dhat_fault_2st + h{i}*L3_fault{i}*( y_attitude-yhat_attitude ); 
end

% UAV Attitude Dynamics (자세 다이나믹스)
for i = 1 : ra
    for j = 1 : ra
       d_attitude     =   d_attitude + h{i}*h{j}*( Aa{i}*x_attitude + Ba{i}*ESC_Control(2:4) + Ba{i}*fault_234 + w_a' );
       dhat_attitude  =   dhat_attitude + h{i}*h{j}*( Aa{i}*xhat_attitude + Ba{i}*ESC_Control(2:4) + 0*Ba{i}*fault_234_hat + L_attitude{i}*(y_attitude - yhat_attitude) );
    end
end

% UAV Altitude Dynamics (고도 다이나믹스)
for i = 1 : 2
    for j = 1 : 2
        d_altitude    =   d_altitude + w{i}*w{j}*( Az{i}*x_altitude + Bz{i}*( ESC_Control(1) - (m*g)/(cos(x_attitude(1))*cos(x_attitude(3))) ) + Bz{i}*ESC_Fault(1) + w_al' );
    end
end

% UAV Position Dynamics (위치 다이나믹스)
for i = 1 : 2
    for j = 1 : 2
        d_position    =   d_position + b{i}*b{j}*( Ap{i}*x_position + Bp{i}*[ux;uy] + Bp{i}*[ESC_Fault(1);ESC_Fault(1)]  + w_p' );
    end
end


% position_control = [sin(x7(5)) cos(x7(5)); -cos(x7(5)) sin(x7(5))]*[sin(x7(1)); cos(x7(1))*sin(x7(3))];

% % Fault Observer Dynamics (펄트 옵저버)
% for i = 1 : ra
%     dhat_fault        =   dhat_fault + h{i}*( L1_fault{i}*( y_attitude-yhat_attitude ) + fault_234_first );
%     dhat_fault_1st    =   dhat_fault_1st + h{i}*( L2_fault{i}*( y_attitude-yhat_attitude ) + fault_234_second );
%     dhat_fault_2st    =   dhat_fault_2st + h{i}*L3_fault{i}*( y_attitude-yhat_attitude ); 
% end
% 
% % UAV Attitude Dynamics (자세 다이나믹스)
% for i = 1 : ra
%     for j = 1 : ra
%         d_attitude    = d_attitude + h{i}*h{j}*( Aa{i}*x_attitude + Ba{i}*Ka{j}*(x_attitude - xr_attitude) + Ba{i}*fault_234 + wa' );
%         dhat_attitude = dhat_attitude + h{i}*h{j}*( Aa{i}*xhat_attitude + Ba{i}*Ka{j}*(xhat_attitude - xr_attitude) + Ba{i}*fault_234_hat + L_attitude{i}*(y_attitude - yhat_attitude) );
%     end
% end
% 
% % UAV Altitude Dynamics (고도 다이나믹스)
% for i = 1 : 2
%     for j = 1 : 2
%         d_altitude = d_altitude + w{i}*w{j}*( Az{i}*x_altitude + Bz{i}*Kz{j}*(x_altitude - xr_altitude) + wal' );
%     end
% end
% 
% % UAV Position Dynamics (위치 다이나믹스)
% for i = 1 : 2
%     for j = 1 : 2
%         d_position = d_position + b{i}*b{j}*( Ap{i}*x_position + Bp{i}*Kp{j}*(x_position - xr_position) + wp' );
%     end
% end


end


