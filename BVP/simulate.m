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

noise_gain = 0.00000;
%noise_gain=0;
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

% Initialize forces
% fx = -spring_force*sin(leg_angle) + contact_n;
% fy = spring_force*cos(leg_angle) + contact_n;

% xd_new = xd + dt*fx/mass;
% yd_new = yd + dt*(-mass*g + fy)/mass;


% RK4 Stepping
h = dt;
% k1 step
fx = -spring_force*sin(leg_angle) + contact_n;
fy = spring_force*cos(leg_angle) + contact_n;
k1_x = fx/mass;
k1_y = (-mass*g + fy)/mass;
xd_k2 = xd + (h/2)*k1_x;
yd_k2 = yd + (h/2)*k1_y;
% RK4 subroutine for leg_state
if ( leg_state == 0 ) % in air
 fl = -hip_torque/leg_moi;
 leg_angled_new = leg_angled + (h/2)*fl;
 leg_angle_k2 = leg_angle + (h/2)*(leg_angled + leg_angled_new)/2;
 leg_angled_k2 = leg_angled_new;
else  % on ground
 x_k2 = x + (h/2)*(xd + xd_k2)/2 + pos_n;   
 y_k2 = y + (h/2)*(yd + yd_k2)/2 + pos_n;
 leg_angle_k2 = atan2( foot_x - x_k2, y_k2 - foot_y );
 leg_angled_k2 = 0;
end
% k2 step
fx = -spring_force*sin(leg_angle_k2) + contact_n;
fy = spring_force*cos(leg_angle_k2) + contact_n;
k2_x = fx/mass;
k2_y = (-mass*g + fy)/mass;
xd_k3 = xd + (h/2)*k2_x;
yd_k3 = yd + (h/2)*k2_y;
% RK4 subroutine for leg_state
if ( leg_state == 0 ) % in air
 fl = -hip_torque/leg_moi;
 leg_angled_new = leg_angled + (h/2)*fl;
 leg_angle_k3 = leg_angle + (h/2)*(leg_angled + leg_angled_new)/2;
 leg_angled_k3 = leg_angled_new;
else  % on ground
 x_k3 = x + (h/2)*(xd + xd_k3)/2 + pos_n;   
 y_k3 = y + (h/2)*(yd + yd_k3)/2 + pos_n;
 leg_angle_k3 = atan2( foot_x - x_k3, y_k3 - foot_y );
 leg_angled_k3 = 0;
end
% k3 step
fx = -spring_force*sin(leg_angle_k3) + contact_n;
fy = spring_force*cos(leg_angle_k3) + contact_n;
k3_x = fx/mass;
k3_y = (-mass*g + fy)/mass;
xd_k4 = xd + (h)*k3_x;
yd_k4 = yd + (h)*k3_y;
% RK4 subroutine for leg_state
if ( leg_state == 0 ) % in air
 fl = -hip_torque/leg_moi;
 leg_angled_new = leg_angled + (h/2)*fl;
 leg_angle_k4 = leg_angle + (h/2)*(leg_angled + leg_angled_new)/2;
 leg_angled_k4 = leg_angled_new;
else  % on ground
 x_k4 = x + (h)*(xd + xd_k4)/2 + pos_n;   
 y_k4 = y + (h)*(yd + yd_k4)/2 + pos_n;
 leg_angle_k4 = atan2( foot_x - x_k3, y_k3 - foot_y );
 leg_angled_k4 = 0;
end
% k4 step
fx = -spring_force*sin(leg_angle_k4) + contact_n;
fy = spring_force*cos(leg_angle_k4) + contact_n;
k4_x = fx/mass;
k4_y = (-mass*g + fy)/mass;
% RK4 total
xd_new = xd + ((1/6)*k1_x + (1/3)*(k2_x+k3_x) + (1/6)*k4_x)*dt;
yd_new = yd + ((1/6)*k1_y + (1/3)*(k2_y+k3_y) + (1/6)*k4_y)*dt;
% Euler update
%xd_new = xd + dt*k1_x;%fx/mass;
%yd_new = yd + dt*k1_y;%(-mass*g + fy)/mass;

% Update x and y assuming constant slope
x = x + dt*(xd + xd_new)/2 + pos_n;
%x = x + dt*(xd_new) + pos_n;
xd = xd_new + vel_n;
y = y + dt*(yd + yd_new)/2 + pos_n;
%y = y + dt*(yd_new) + pos_n;
yd = yd_new + vel_n;

% Body angle and leg state
body_angled_new = body_angled + dt*hip_torque/body_moi + angle_n;
body_angle = body_angle + dt*(body_angled + body_angled_new)/2;
body_angled = body_angled_new;
if ( leg_state == 0 ) % in air
 leg_angled_new = leg_angled - dt*hip_torque/leg_moi;
 leg_angle = leg_angle + dt*(leg_angled + leg_angled_new)/2;
 %leg_angle = leg_angle + dt*(leg_angled);
 leg_angled = leg_angled_new;
else  % on ground
 leg_angle = atan2( foot_x - x, y - foot_y );
 leg_angled = 0;
end

time = time + dt;

% For the spring/damper model of the ground should I be modeling the 
% slip conditions or just the force over time?

%xd_new = xd + dt*fx/mass;
% h = dt;
% k1 = fx(xd)/mass;
% xd_k2 = xd + (h/2)*k1;
% k2 = fx(xd_k2)/mass;
% xd_k3 = xd + (h/2)*k2;
% k3 = fx(xd_k3)/mass;
% xd_k4 = xd + (h)*k3;
% k4 = fx(xd_k4)/mass;
% xd_new = xd + ((1/6)*k1 + (1/3)*(k2+k3) + (1/6)*k4)*h;

% x = x + dt*(xd + xd_new)/2 + pos_n;
% xd = xd_new + vel_n;

%yd_new = yd + dt*(-mass*g + fy)/mass;
% y = y + dt*(yd + yd_new)/2 + pos_n;
% yd = yd_new + vel_n;

result = y;

