%% Initializing

l0 = 0.22; %default "combined" leg length
l1 = 0.1; % upper leg section length
l2 = 0.2; % lower leg section length
ml = 0.15; %mass leg
mb = 2; %mass of body kg
m = mb + ml;
rb = 0.4; %radius of rxn wheel on body
Ib = 0.5*rb^2*mb; % body inertia
%Il = 1/3.0*0.15*l^2; %inertia of the leg -- neglecting

%% Update Eqs
% th1 = pi/2 - (leg_angle - body_angle) - acos((leg_length^2 + l1^2 - l2^2)/(2*l1*leg_length)) - body_angle;
% th2 = pi/2 - (leg_angle - body_angle) + acos((leg_length^2 + l1^2 - l2^2)/(2*l1*leg_length)) - body_angle;
% beta = th2-th1;
% q1 = pi/2-th2;
% q2 = pi/2-th1;
% foot_x = x + l1*sin(q1) + l2*sdel;
% foot_y = y - l1*cos(q1) - l2*cdel;
% leg_length_new = sqrt( (x - foot_x)^2 + (y - foot_y)^2 );

%% EOMS
% Linearized
A = [0 1 0 0 0 0; 0 0 0 0 0 0; 0 0 0 1 0 0; 0 0 0 0 0 0; 0 0 0 0 0 1; 0 0 0 0 0 0];
B = [0 0; 1/m 0; 0 0; 0 1/(m*l0); 0 0; 0 -1/Ib];
C = eye(6); %able to measure all control vars and derivs
D = zeros(6,2);
sys_ss = ss(A,B,C,D); %can add Ts on end to get discrete
step(sys_ss)

%% TFs
[num, den] = ss2tf(A,B,C,D, 2) % for l -- TF found from iu'th input

