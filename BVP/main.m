% simulate a Raibert hopper

% allocate global variables
global h_axes body leg min_x max_x
global dt time x y xd yd 
global hip_torque leg_angle body_angle leg_angled body_angled
global leg_state foot_x foot_y leg_lengthd leg_length rest_leg_length
global control_state height_desired leg_angle_desired last_bounce_time
global last_touchdown_time last_takeoff_time max_height last_max_height
global speed_desired
global leg11 leg12 leg21 leg22 leg_torque
global th1 th2 l1 l2 T1 T2 Fx Fy Torq flight_time

figure(1) % choose right plot target

% intialize variables.
% stuff we want to control
height_desired = 0.8;
speed_desired = 0.4;

% constants
dt = 0.0001;
l1 = 0.2;
l2 = 0.4;

% initial state of robot
time = 0.0;
x = 0.0;
y = 1.0;
xd = 0.0;
yd = 0.0;
body_angle = 0;
leg_angle = 0.0;
body_angled = 0;
leg_angled = 0;
hip_torque = 0;
leg_state = 0; % simulator state: leg not on ground
% controller state
control_state = 0; 
last_bounce_time = 0.25;
last_touchdown_time = -1;
last_takeoff_time = -1;
max_height = y;
last_max_height = y;
leg_length = 0.5;
Torq = [0;0];
flight_time = 0.3

max_n_points = 1000;

% allocate array
array(max_n_points,17) = 0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% outer loop: save data and draw picture at slow rate
for i = 1:5000

% simulate and control at faster rate
 for j = 1:20
  control();
  simulate();
 end;

% save data in array
 if ( i <= max_n_points )
   array(i,1) = time;
   array(i,2) = y;
   array(i,3) = yd;
   array(i,4) = control_state;
   array(i,5) = body_angle;
   array(i,6) = leg_angle;
   array(i,7) = hip_torque;
   array(i,8) = leg_angle_desired;
   array(i,9) = xd;
   array(i,10) = Fx;
   array(i,11) = Fy;
   array(i,12) = Torq(1);
   array(i,13) = Torq(2);
   array(i,14) = leg_length;
   array(i,15) = leg_lengthd;
   array(i,16) = leg_angled;
   array(i,17) = body_angled;
   array(i,18) = flight_time;
 end;

% hack to keep it in view
 if ( x > max_x )
  x = x - (max_x - min_x); 
  foot_x = foot_x - (max_x - min_x); 
 end;

 if ( x < min_x )
  x = x + (max_x - min_x); 
  foot_x = foot_x + (max_x - min_x); 
 end;
 draw();

% have we crashed?
 if y < 0.1
   break;
 end;

% keep track of how long before crash.
 if ( i <= max_n_points )
   max_i = i; 
 end;
end
