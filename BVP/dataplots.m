%%settings
control_plots = 1; %1 to print T/F
main_plots = 1;
vars_plots = 1;

%%plots
if(main_plots)
    figure;
    plot(array(1:max_i,1),array(1:max_i,2))
    ylabel('Hop Height')

    figure;
    plot(array(1:max_i,1),array(1:max_i,7))
    ylabel('Hip Torque')

    figure;
    plot(array(1:max_i,1),array(1:max_i,6),'b', ...
     array(1:max_i,1),array(1:max_i,8),'r')
    legend('Leg Angle', 'Leg Angle Desired')

    figure;
    plot(array(1:max_i,1),array(1:max_i,9))
    ylabel('Horizontal Speed')
end

if(control_plots)
    figure;
    plot(array(1:max_i,1),array(1:max_i,10),'b', array(1:max_i,1),array(1:max_i,11),'r')
    legend('X-force', 'Y-force')

    figure;
    plot(array(1:max_i,1),array(1:max_i,12),'b', array(1:max_i,1),array(1:max_i,13),'r')
    legend('th1 torque', 'th2 torque')
    axis([1 10 -100 100])
end

if(vars_plots)
    figure;
    plot(array(1:max_i,1),array(1:max_i,5),'b', array(1:max_i,1),array(1:max_i,6),'r')
    legend('body_angle', 'leg_angle')

    figure;
    plot(array(1:max_i,1),array(1:max_i,17),'b', array(1:max_i,1),array(1:max_i,16),'r')
    legend('d_body_angle', 'd_leg_angle')
    
    figure;
    plot(array(1:max_i,1),array(1:max_i,14))
    xlabel('leg length')
    
    figure;
    plot(array(1:max_i,1),array(1:max_i,15))
    xlabel('d_leg_length')
end

% array(i,1) = time;
% array(i,2) = y;
% array(i,3) = yd;
% array(i,4) = control_state;
% array(i,5) = body_angle;
% array(i,6) = leg_angle;
% array(i,7) = hip_torque;
% array(i,8) = leg_angle_desired;
% array(i,9) = xd;
% array(i,10) = Fx;
% array(i,11) = Fy;
% array(i,12) = Torq(1);
% array(i,13) = Torq(2);
% array(i,14) = leg_length;
% array(i,15) = leg_lengthd;
% array(i,16) = leg_angled;
% array(i,17) = body_angled;