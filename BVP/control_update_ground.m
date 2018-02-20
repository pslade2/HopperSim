function [u1_step, u2_step, t, uv] = control_update(dt, time, td, to, ls, gs, ps, control_state, last_bounce_time, plot, control_num, leg_angle_desired,u0);
    %control state vars
%     in_air = 1;
%     on_ground_going_down = 2;
%     on_ground_going_up = 3;
    %[leg_length, leg_lengthd, leg_length_dd] = ls;
    %[leg_angle, leg_angled, leg_angle_dd] = gs;
    %[body_angle, body_angled, body_angle_dd] = ps;

    if(control_state == 1)
        d_des_leg_length = 0.0;
        dd_des_leg_length = 0.0;
        d_des_body_angle = 0.0;
        dd_des_body_angle = 0.0;
        d_des_leg_angle = 0.0;
        dd_des_leg_angle = 0.0;
        d_ds_leg_length = 0.0;
        dd_ds_leg_length = 0.0;
        d_ds_body_angle = 0.0;
        dd_ds_body_angle = 0.0;
        d_ds_leg_angle = 0.0;
        dd_ds_leg_angle = 0.0;
        psi0 = 0;
        %ls = 0.5;
        %gs = -0.25;
        psif = 0;
        le = 0.5;
        ge = 0.22;
        % in air so from bounce time subtract time - takeofftime
%         t0 = last_bounce_time - (time - to)
        t0 = time - to;
        tf = last_bounce_time;
%         if(tf < 0)
%             tf = 0;
%         end
    else
        d_des_leg_length = 2.2;
        dd_des_leg_length = 0.0;
        d_des_body_angle = 0.0;
        dd_des_body_angle = 0.0;
        d_des_leg_angle = 0.0;
        dd_des_leg_angle = 0.0;
        d_ds_leg_length = -2.6;
        dd_ds_leg_length = 0.0;
        d_ds_body_angle = 0.48;
        dd_ds_body_angle = 0.0;
        d_ds_leg_angle = 0.0;
        dd_ds_leg_angle = 0.0;
        %ls = 0.5;
        le = 0.5;
        %gs = -0.08;
        ge = 0;
        t0 = time - to;
        tf = last_bounce_time;
%         tf = last_bounce_time - (time - td);
%         if(tf < 0)
%             tf = 0;
%         end
    end

%     l_ic = [ls d_des_leg_length dd_des_leg_length];%[leg_length, leg_lengthd, leg_length_dd, d_des_leg_length, dd_des_leg_length];
%     psi_ic = [ps d_des_body_angle dd_des_body_angle];
%     gamma_ic = [gs  d_des_leg_angle dd_des_leg_angle];
    l_ic = [ls le d_des_leg_length dd_des_leg_length];
    psi_ic = [ps d_des_body_angle dd_des_body_angle];
    gamma_ic = [gs  d_des_leg_angle dd_des_leg_angle];
    ge = leg_angle_desired;
    bcs = [ls, le, gs, ge, d_ds_leg_length, d_des_leg_length, ...
        d_ds_leg_angle, d_des_leg_angle, ...
        dd_ds_leg_length, dd_des_leg_length,...
        dd_ds_leg_angle, dd_des_leg_angle];

    %tf = last_bounce_time; %Can sub in the equation for this 
    if control_state == 1;
    t0 = 0.0;
    [u1_step,u2_step, t] = bvp_fun( bcs );
    uv{1} = u1_step; uv{2} = u2_step;
    t = t*td;
    diffv = abs(time-t); [val,tind] = min(diffv); 
    u1_step = u1_step(1); u2_step = u2_step(1);
    else
    %u1 = F/m, u2 = Tau/m, set internally to 100, dt = tf/100
    %[u1_step,u2_step] = trajectory_fun_mpc(tf, t0, dt, gamma_ic, psi_ic, l_ic, leg_angle_desired,u0);%( tf, t0, dt, gamma_ic,  psi_ic, l_ic );
    l_ic = [ls d_des_leg_length dd_des_leg_length];%[leg_length, leg_lengthd, leg_length_dd, d_des_leg_length, dd_des_leg_length];
    psi_ic = [ps d_des_body_angle dd_des_body_angle];
    gamma_ic = [gs  d_des_leg_angle dd_des_leg_angle];
    %[tv, u1_traj, u2_traj, gamma_traj, psi_traj, l_traj] = trajectory_fun(td, 1.0, gamma_ic,  psi_ic, l_ic, plot);
    [u1_step,u2_step] = trajectory_fun_mpc(tf, t0, dt, gamma_ic, psi_ic, l_ic, leg_angle_desired,u0);%( tf, t0, dt, gamma_ic,  psi_ic, l_ic );
    %u1_step = u1_traj(1);
    %u2_step = u2_traj(1);
    t = []; uv = [];
    end
    %u1_step = u1_traj(1)
    %u2_step = u2_traj(1)
end