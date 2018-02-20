function res = bcvect(za,zb, bcs);
% parameter phi: phi_ic = [phi(0); phi(tf); dphi(0); ddphi(0); dphi(tf); ddphi(tf)]
B = [0, 0; 0, 0; 0, 0; 0, 0; 1, 0; 0, 1];
%bcs = [0 0 0 0 0 0];
A = [0 0 1 0 0 0; 0 0 0 1 0 0; 0 0 0 0 1 0; 0 0 0 0 0 1;...
    0 0 0 0 0 0; 0 0 0 0 0 0];
ic = [0, 0, 0, 0,...
    0, 0, 0, 0, ...
    0, 0, 0, 0];
psi0 = 0;
l0 = 0.5;
gamma0 = -0.25;
psif = 0;
lf = 0.5;
gammaf = 0.22;
% bcs = [l0, lf, gamma0, gammaf, dl0, dlf, dgamma0, dgammaf, ddl0, ddlf,...
%     ddgamma0, ddgammaf];
% bcs = [l0, lf, gamma0, gammaf, 0, 0, 0, 0, 0, 0,...
%     0, 0];


% w = -zb(7:12)'*B/2; w=w';
% res = [za(1) - bcs(1); zb(1) - bcs(2);...
%     za(2) - bcs(3); zb(2) - bcs(4);...
%     za(3) - bcs(5); zb(3) - bcs(6);... % d/dt
%     za(4) - bcs(7); zb(4) - bcs(8);... %d/dt
%     za(5) - bcs(9); zb(5) - bcs(10);... %d2/dt2
%     za(6) - bcs(11)]; zb(6) - bcs(12)];%... %d2/dt2
%     %w'*w + zb(7:12)'*A*zb(1:6) + zb(7:12)'*B*w];
% %res
res = [za(1) - bcs(1); zb(1) - bcs(2);...
    za(2) - bcs(3); zb(2) - bcs(4);... %d/dt
    za(3) - bcs(5); zb(3) - bcs(6)];
    
    
    


end
