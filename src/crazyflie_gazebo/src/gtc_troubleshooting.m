clear
clc
close all




%% GTC Assumption
c_tf = 0.0061;
kf = 2.2e-8;

% Figure out what the Gazebo motor constants mean compared to these

%% System Params
m = 0.026 + 0.00075*4; % kg
g = 9.8066; % m/s^2

mg = m*g; % N

d = 0.040; % m 
dp = d*sin(pi/4); % m



Gamma = [1  1  1  1;
         dp  dp -dp -dp; 
        -dp  dp  dp -dp;
        c_tf -c_tf c_tf -c_tf];
    
Gamma_I = inv(Gamma);    
    
J = [1.65717e-5,0,0;
    0,1.66556e-5,0;
    0,0,2.92617e-5];

e3 = [0,0,1]';


%% Desired Trajectories
x_d = [0,0,1.0]';
v_d = [0,0,0]';
a_d = [0,0,0]';

omega_d = [0,20,0]';
domega_d = [0,0,0]';

b1_d = [1,0,0]';



%% Control Params
kpx = 0.1*0;
kdx = 0.08*0;
kpR = 0.04*0;
kdR = 0.0005;

thrust_flag = 0;


%% Current State
pos = [0 0 0.8039]';
vel = [0 0 0]';
omega = [0 0 0]';

eul = [0,pi/18,0]
R = eul2rotm(eul,'ZYX')

% R = [0.9961947,  0.0000000,  0.0871557;
%    0.0000000,  1.0000000,  0.0000000;
%   -0.0871557,  0.0000000,  0.9961947];
% R = eye(3,3)







%% Translational Errors
ex = pos-x_d;
ev = vel-v_d;

A = -kpx*ex-kdx*ev + m*g*e3 + m*a_d;

u1 = dot(A,R*e3)*thrust_flag;



%% Desired Attitude

b3_d = A/norm(A);
b2_d = cross(b3_d,b1_d);


R_d = [cross(b2_d,b3_d),b2_d,b3_d]

R_1 = [0.9961947,  0.0000000,  0.0871557;
   0.0000000,  1.0000000,  0.0000000;
  -0.0871557,  0.0000000,  0.9961947]; % 5 deg pitch
% 
% R_2 = [0.9848077,  0.0000000,  0.1736482;
%    0.0000000,  1.0000000,  0.0000000;
%   -0.1736482,  0.0000000,  0.9848077] % 10 deg pitch
% 
% R_3 = [  1.0000000,  0.0000000,  0.0000000;
%    0.0000000,  0.9848077, -0.1736482;
%    0.0000000,  0.1736482,  0.9848077 ] % 10 deg roll
% 
% R_4 = [  0.9848077, -0.1736482,  0.0000000;
%    0.1736482,  0.9848077,  0.0000000;
%    0.0000000,  0.0000000,  1.0000000 ] % 10 deg yaw
% 
% R_d = R_1


%% Rotational Errors
eR = 0.5*dehat(R_d'*R-R'*R_d)'
eR_deg = eR*180/pi
e_omega = omega - R'*R_d*omega_d
e_omega = omega - omega_d
u2 = -kpR*eR - kdR*e_omega %+ cross(omega,J*omega) - J*(hat(omega)*R'*R_d*omega_d - R'*R_d*domega_d);
% u2_1 = cross(omega,J*omega) - J*(hat(omega)*R'*R_d*omega_d - R'*R_d*domega_d)


%% Motor Speeds
FM = [u1;u2]


f_a = Gamma_I .* FM(:,ones,1,size(FM,2))' % f matrix before addition
f = Gamma_I*FM

% fMin = 0;
% f(f<fMin) = fMin;

motorspeed_d = sqrt(1/kf*f)








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
































