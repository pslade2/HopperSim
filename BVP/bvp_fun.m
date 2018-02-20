function [u1,u2,z, t] = ...
    bvp_fun( bcs, leg_length)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

% Parameters
g = 9.81;
% bcs = [l0, lf, gamma0, gammaf, dl0, dlf, dgamma0, dgammaf, ddl0, ddlf,...
%     ddgamma0, ddgammaf];

%warning off;
% if bcs(4) > pi/2;
%     bcs(4) = bcs(4)+pi;
% end
% if bcs(4) < -pi/2;
%     bcs(4) = bcs(4)-pi;
% end
solinit = bvpinit(linspace(bcs(end-2),bcs(end-1),...
    round((bcs(end-1)-bcs(end-2))/bcs(end))),...
    [0 0 0 1 1 1]);
sol = bvp4c(@odevect, @(za, zb) bcvect(za,zb, bcs(1:end-2)),solinit);
z = sol.y; t = sol.x;
%u1 = z(5,:) - z(1,:).*z(4,:).^2 + g*cos(z(2,:));
% for i = 1:length(z(2,:));
%    if z(2,i) < 0 && z(2,i) > -2*pi;
%        z(2,i) = z(2,i) + pi;
%    end
% end
%u2 = (z(1,:) + 2*z(3,:).*z(4,:)./z(1,:)).*z(1,:).^2;
I = 0.1; mass = I / (leg_length^2);
u2 = mass * leg_length^2 * z(3,:);
u1 = zeros(length(u2));
% - g*sin(z(2,:))./z(1,:)


end

