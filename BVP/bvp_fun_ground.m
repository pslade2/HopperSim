function [u1,u2, z, t] = ...
    bvp_fun_ground( bcs )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

% Parameters
g = 9.81;
% bcs = [l0, lf, gamma0, gammaf, dl0, dlf, dgamma0, dgammaf, ddl0, ddlf,...
%     ddgamma0, ddgammaf];

%warning off;
%solinit = bvpinit(linspace(0,1,20), [bcs(1) 0 0 1 0 1 1 1 1 1 1 1 1]);
%sol = bvp4c(@odevect,@(za, zb) bcvect_ground(za,zb, bcs),solinit);

%
% solinit = bvpinit(linspace(bcs(end-2),bcs(end-1),...
%     round((bcs(end-1)-bcs(end-2))/bcs(end))),...
%     [bcs(1) 0 0 1 0 1 1 1 1 1 1 1 1]);
% bcs = [ls, le, gs, ge, d_ds_leg_length, d_des_leg_length,...
%         d_ds_leg_angle, d_des_leg_angle, ...
%         dd_ds_leg_length, dd_des_leg_length,...
%         dd_ds_leg_angle, dd_des_leg_angle];
warning on;
solinit = bvpinit(linspace(0,1,100), [bcs(1) 0 0 bcs(7)...
    bcs(9) bcs(11) 1 1 1 1 1 1 1]);
sol = bvp4c(@odevect_ground, @(za, zb) bcvect_ground(za,zb, bcs(1:end-2)),solinit);
%

z = sol.y; t = sol.x * z(13,end);
I = 0.1; mass = I ./ (z(1,:).^2);
u1 = z(5,:) - z(1,:).*z(4,:).^2 + g*cos(z(2,:));
u2 = (z(6,:) - g*sin(z(2,:))/z(1,:) + 2*z(3,:).*z(4,:)./z(1,:)).*z(1,:).^2;
u1 = u1.*mass;
u2 = u2.*mass;


end

