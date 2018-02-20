function pos = draw()
% draw hopper

global h_axes body leg min_x max_x
global dt time x y xd yd 
global hip_torque leg_angle body_angle leg_angled body_angled
global leg_state foot_x foot_y leg_lengthd leg_length rest_leg_length
global leg11 leg12 leg21 leg22
global th1 th2 l1 l2 T1 T2

body_length = 0.2;
bdx = 0.5*body_length*cos( body_angle );
bdy = 0.5*body_length*sin( body_angle );

%leg 1, upper and lower are 1 and 2
leg1x1 = l1*cos(th1);
leg1y1 = -l2*sin(th1);
% leg2
leg2x1 = -l1*sin(th2-3.14159/2);
leg2y1 = -l2*cos(th2-3.14159/2);

set(body,'Parent',h_axes,'Xdata',[x - bdx x + bdx], ...
'Ydata',[y - bdy y + bdy],'visible','on');
set(leg,'Parent',h_axes,'Xdata',[x foot_x], ...
'Ydata',[y foot_y],'visible','on');

%added
set(leg21,'Parent',h_axes,'Xdata',[x x+leg2x1], ...
'Ydata',[y y+leg2y1],'visible','on');
set(leg22,'Parent',h_axes,'Xdata',[x+leg2x1 foot_x], ...
'Ydata',[y+leg2y1 foot_y],'visible','on');
set(leg11,'Parent',h_axes,'Xdata',[x x+leg1x1], ...
'Ydata',[y y+leg1y1],'visible','on');
set(leg12,'Parent',h_axes,'Xdata',[x+leg1x1 foot_x], ...
'Ydata',[y+leg1y1 foot_y],'visible','on');

drawnow

end
