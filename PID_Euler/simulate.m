function result = simulate()
% simulate a Raibert hopper

global h_axes body leg
global dt time x y xd yd 
global hip_torque leg_angle body_angle leg_angled body_angled
global leg_state foot_x foot_y leg_lengthd leg_length rest_leg_length
global leg11 leg12 leg21 leg22 leg_torque
global th1 th2 l1 l2 T1 T2 Fx Fy
global leg_length_default leg_k leg_damping

mass = 1.0;
g = 9.81;

leg_k = 200;
leg_damping = 1.0;

noise_gain = 0.00002;
pos_n = noise_gain*normrnd(0,1);% normrnd(MU,SIGMA,M,N,...) gives M-by-N-by- array.
vel_n = noise_gain*normrnd(0,1);
angle_n = noise_gain*normrnd(0,1);
contact_n = noise_gain*normrnd(0,1);


body_moi = 1.0;
leg_moi = 0.1;

foot_y_new = y - rest_leg_length*cos( leg_angle );
foot_x_new = x + rest_leg_length*sin( leg_angle );

if foot_y_new > 0
 leg_state = 0; % in air
 spring_force = 0;
 foot_x = foot_x_new;
 foot_y = foot_y_new;
 leg_length = rest_leg_length;
else
 if leg_state == 0
   foot_x = foot_x_new;
   foot_y = 0;
   leg_state = 1; % we are on the ground
 end;
 leg_length = sqrt((x - foot_x)^2 + (y - foot_y)^2);
 leg_lengthd = ((x - foot_x)*xd + (y - foot_y)*yd)/leg_length;
 spring_force = leg_k*(rest_leg_length - leg_length) - leg_damping*leg_lengthd;
end

%%Contact and Dynamics Modeling
%Adding spring/damper for ground contact??

fx = -spring_force*sin(leg_angle) + contact_n;
fy = spring_force*cos(leg_angle) + contact_n;

time = time + dt;

% For the spring/damper model of the ground should I be modeling the 
% slip conditions or just the force over time?

xd_new = xd + dt*fx/mass;
% h = dt;
% k1 = fx(xd)/mass;
% xd_k2 = xd + (h/2)*k1;
% k2 = fx(xd_k2)/mass;
% xd_k3 = xd + (h/2)*k2;
% k3 = fx(xd_k3)/mass;
% xd_k4 = xd + (h)*k3;
% k4 = fx(xd_k4)/mass;
% xd_new = xd + ((1/6)*k1 + (1/3)*(k2+k3) + (1/6)*k4)*h;

x = x + dt*(xd + xd_new)/2 + pos_n;
xd = xd_new + vel_n;

yd_new = yd + dt*(-mass*g + fy)/mass;
y = y + dt*(yd + yd_new)/2 + pos_n;
yd = yd_new + vel_n;

body_angled_new = body_angled + dt*hip_torque/body_moi + angle_n;
body_angle = body_angle + dt*(body_angled + body_angled_new)/2;
body_angled = body_angled_new;

if ( leg_state == 0 ) % in air
 leg_angled_new = leg_angled - dt*hip_torque/leg_moi;
 leg_angle = leg_angle + dt*(leg_angled + leg_angled_new)/2;
 leg_angled = leg_angled_new;
else  % on ground
 leg_angle = atan2( foot_x - x, y - foot_y );
 leg_angled = 0;
end

result = y;

