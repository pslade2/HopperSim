function dzdt = odevect_ground(tau,z);
A = [0 0 1 0 0 0; 0 0 0 1 0 0; 0 0 0 0 1 0; 0 0 0 0 0 1;...
    0 0 0 0 0 0; 0 0 0 0 0 0];
B = [0, 0; 0, 0; 0, 0; 0, 0; 1, 0; 0, 1];
w = -z(7:12)'*B/2;
dxdt = A*z(1:6) + B*w';
dpdt = -A'*z(7:12);
dzdt = [dxdt; dpdt];
%dzdt = z(13)*[z(3); z(4); z(5); z(6); w(1); w(2); dpdt; 0];
dzdt = z(13)*[dxdt; dpdt; 0];
end
    