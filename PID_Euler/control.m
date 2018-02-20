function result = control1()
% control vertical hopper

global h_axes body leg
global dt time x y xd yd
global hip_torque leg_angle body_angle leg_angled body_angled
global leg_state foot_x foot_y leg_lengthd leg_length rest_leg_length
global control_state height_desired leg_angle_desired last_bounce_time
global last_touchdown_time last_takeoff_time max_height last_max_height
global speed_desired
global leg11 leg12 leg21 leg22 leg_torque
global th1 th2 l1 l2 T1 T2 Fx Fy Torq

%forward kin - find foot pos
%know compressed distance of the spring for desired force
%then use jacobian to get the torques at motor to apply that force
%How do the theta's propogate? How do I add the hip control as well?

Fx = 0;
Fy = 0;
% control_state values
init = 0;
in_air = 1;
on_ground_going_down = 2;
on_ground_going_up = 3;

hip_air_k = 5; %10
hip_air_b = 1.5;
hip_grnd_k = 5; %10
hip_grnd_b = 1;

leg_length_default = 0.5;

leg_length_gain = 0.9;
angle_gain = 0.05; 

rest_leg_length = leg_length_default;
%leg_length = leg_length_default;
hip_torque = 0;   

%% Kinematics update
%update leg angles
th1 = pi/2 - (leg_angle - body_angle) - acos((leg_length^2 + l1^2 - l2^2)/(2*l1*leg_length)) - body_angle;
th2 = pi/2 - (leg_angle - body_angle) + acos((leg_length^2 + l1^2 - l2^2)/(2*l1*leg_length)) - body_angle;
beta = th2-th1;
q1 = pi/2-th2;
q2 = pi/2-th1;

%constants
cdel = -cos(q1 - acos(sqrt((1-cos(beta))/2)) - acos((l1/l2)*sqrt((1-cos(beta))/2)));
sdel = -sin(q1 - acos(sqrt((1-cos(beta))/2)) - acos((l1/l2)*sqrt((1-cos(beta))/2)));

%foot coordinates and leg length
foot_x_kin = x + l1*sin(q1) + l2*sdel;
foot_y_new = y - l1*cos(q1) - l2*cdel;
leg_length_new = sqrt( (x - foot_x)^2 + (y - foot_y)^2 );
leg_lengthd = ((x - foot_x)*xd + (y - foot_y)*yd)/leg_length;

%Virtual spring consants
leg_k = 200.0;
leg_damping = 1.0;

%Jacobian
A = 1/(2*sin(beta));
D = (l1/(2*l2*sqrt(2))*(1/sqrt(1-cos(beta)))*(1/sqrt((1-l1/l2)*(1-cos(beta)/2))));
%J = [-l1*sin(q1)-l2*(1-(A+D))*sdel -l2*(A+D)*sdel; l1*cos(q1)+l2*(1-(A+D))*cdel l2*(A+D)*cdel];
J = [l1*cos(q1)+l2*(1-(A+D))*cdel l2*(A+D)*cdel; -1*(-l1*sin(q1)-l2*(1-(A+D))*sdel) -1*(-l2*(A+D)*sdel)];

%% State Machine
% initialization
if control_state == init
  control_state = in_air;
  result = control_state;
  return;
end;

if control_state == in_air
  if foot_y_new < 0
    last_touchdown_time = time;
    if yd <= 0
      control_state = on_ground_going_down;
    else
      control_state = on_ground_going_up;
    end;
    result = control_state;
    return;
  end;

% angle_lim = 200;
  Ts = last_bounce_time;
  xf = xd*Ts/2 + angle_gain*(xd - speed_desired);
  leg_angle_desired = atan2( xf, y - foot_y );
  %leg_angle_desired = atan2(xf - x, y);
%   if leg_angle_desired > angle_lim
%       leg_angle_desired = angle_lim;
%   elseif leg_angle_desired < -angle_lim
%       leg_angle_desired = -angle_lim;
%   end
  
  hip_torque = hip_air_k*(leg_angle - leg_angle_desired) + ...
                             hip_air_b*leg_angled;
  if ( y > max_height )
    max_height = y;
  end;
  if ( yd < 0 )
    last_max_height = max_height;
  end;
end;

if control_state == on_ground_going_down
  if leg_length_new > rest_leg_length
    control_state = in_air;
    max_height = y;
    result = control_state;
    last_takeoff_time = time;
    return;
  end;
  if yd > 0
    control_state = on_ground_going_up;
    result = control_state;
    return;
  end;
  spring_force = leg_k*(rest_leg_length - leg_length_new) - leg_damping*leg_lengthd;
  Fx = -spring_force*sin(leg_angle);
  Fy = spring_force*cos(leg_angle);
    
  %Torques to achieve these forces
  Torq = J.'*[Fx; Fy];
  hip_torque = hip_grnd_k*(-body_angle) - hip_grnd_b*body_angled;
end;

if control_state == on_ground_going_up
    %Figure out leg length for spring force
    if last_max_height < height_desired
        rest_leg_length = leg_length_default + (height_desired - last_max_height)*leg_length_gain
    end
  
    %Desired spring forces
    spring_force = leg_k*(rest_leg_length - leg_length_new) - leg_damping*leg_lengthd;
    Fx = -spring_force*sin(leg_angle)
    Fy = spring_force*cos(leg_angle)
    
    %Torques to achieve these forces
    Torq = J.'*[Fx; Fy]
  
  if leg_length_new > rest_leg_length
    control_state = in_air;
    max_height = y;
    result = control_state;
    last_takeoff_time = time;
    if ( last_touchdown_time > 0 )
      last_bounce_time = last_takeoff_time - last_touchdown_time;
    end;
    return;
  end;
  if yd < 0
    control_state = on_ground_going_down;
    result = control_state;
    return;
  end;
  hip_torque = hip_grnd_k*(-body_angle) - hip_grnd_b*body_angled;
end;
