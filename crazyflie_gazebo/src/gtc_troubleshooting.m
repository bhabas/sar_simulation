clear
clc
close all


%% System Params
m = 0.027; % kg
g = 9.8066; % m/s^2

mg = m*g; % N

d = 0.040; % m 
dp = d*sin(pi/4); % m

kf = 2.2e-8; % Motor thrust coefficient
c_tf = 0.0061; % Thrust Moment coefficient



Gamma = [1  1  1  1;
         dp  dp -dp -dp; 
        -dp  dp  dp -dp;
        c_tf -c_tf c_tf -c_tf];
    
Gamma_I = inv(Gamma);    
    
J = [1.65717e-5,0,0;
    0,1.66556e-5,0;
    0,0,2.92617e-5]; % Intertia Matrix

e3 = [0,0,1]';

%% Control Params
kp_x = 0.7;
kd_x = 0.250;
kp_R = 0.004;
kd_R = 0.0008;

%% Current State
pos = [0 0 0]';
vel = [0 0 0]';
omega = [0 0 1]';

% eul = [-0.5221,-0.1464,5.3115]*pi/180; 
% R = eul2rotm(eul,'XYZ')

R = [0.99619,-0.08715,0;
    0.08715,0.99619,0.0;
    0,0,1]

%% Desired Trajectories
x_d = [0,0,1.0]';
v_d = [0,0,0]';
a_d = [0,0,0]';

omega_d = [0,0,0]';
domega_d = [0,0,0]';

b1_d = [1,0,0]';




%% Thrust Calculation
e_x = pos-x_d;
e_v = vel-v_d;
PID_X = -kp_x*e_x - kd_x*e_v
A =  PID_X + m*g*e3 + m*a_d;
u1 = dot(A,R*e3);


%% Desired Attitude
b3_d = A/norm(A);
b2_d_temp = cross(b3_d,b1_d);
b2_d = b2_d_temp/norm(b2_d_temp);
R_d = [cross(b2_d,b3_d),b2_d,b3_d]




%% Moment Calculation
e_R = 0.5*dehat(R_d'*R-R'*R_d)'
eR_deg = e_R*180/pi;
e_omega = omega - R'*R_d*omega_d;
e_omega = omega - omega_d;
PID_R = -kp_R*e_R - kd_R*e_omega

Gyro_dyn = cross(omega,J*omega) - J*(hat(omega)*R'*R_d*omega_d - R'*R_d*domega_d)
u2 = PID_R + Gyro_dyn

%% Thrust/Moment
FM = [u1;u2]

%% 1.) Motor Speeds from Thrust/Moment
% f1 = Gamma_I*FM
% f1_a = Gamma_I .* FM(:,ones,1,size(FM,2))'; % broken up f matrix before addition
% 
% motorspeed1 = sqrt(1/kf*f1)



%% Motorspeeds from PWM


% Motor Max Thrust: 0.1375 N = 2500^2*kf
% Max Roll/Pitch Moment: 0.0078 Nm = 2*0.1375*dp
% Max Yaw Moment: 0.00016775 Nm = 2*c_tf*0.1375

%% Thrusts per motor
f_thrust = u1/4;
f_roll  = u2(1)/(4*dp); % divide thrust by 4 because some will increase and others decrease
f_pitch = u2(2)/(4*dp);
f_yaw = u2(3)/(4*c_tf);

f2(1,1) = f_thrust + f_roll - f_pitch + f_yaw;
f2(2,1) = f_thrust + f_roll + f_pitch - f_yaw;
f2(3,1) = f_thrust - f_roll + f_pitch + f_yaw;
f2(4,1) = f_thrust - f_roll - f_pitch - f_yaw;


%% Firmware Thrust Calc
f_thrust_pwm = (f_thrust/0.1375)*65535.0
f_roll_pwm = (f_roll/0.1375)*65535.0
f_pitch_pwm = (f_pitch/0.1375)*65535.0
f_yaw_pwm = (f_yaw/0.1375)*65535.0


f_pwm(1,1) = f_thrust_pwm + f_roll_pwm - f_pitch_pwm + f_yaw_pwm;
f_pwm(2,1) = f_thrust_pwm + f_roll_pwm + f_pitch_pwm - f_yaw_pwm;
f_pwm(3,1) = f_thrust_pwm - f_roll_pwm + f_pitch_pwm + f_yaw_pwm;
f_pwm(4,1) = f_thrust_pwm - f_roll_pwm - f_pitch_pwm - f_yaw_pwm

MS = sqrt((f_pwm*(0.1375/65535))*1/kf)





function [v_hat] = hat(v)
    v_hat(3,2) = v(1);
    v_hat(2,3) = -v(1);
    
    v_hat(1,3) = v(2);
    v_hat(3,1) = -v(2);
    
    v_hat(1,2) = -v(3);
    v_hat(2,1) = v(3);

end

function [v] = dehat(v_hat)
    v(1) = v_hat(3,2);
    v(2) = v_hat(1,3);
    v(3) = v_hat(2,1);
end
































