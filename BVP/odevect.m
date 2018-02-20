function dzdt = odevect(tau,z);
A = [0 0 1 0 0 0; 0 0 0 1 0 0; 0 0 0 0 1 0; 0 0 0 0 0 1;...
    0 0 0 0 0 0; 0 0 0 0 0 0];
A = [0 1 0; 0 0 1; 0 0 0];
B = [0; 0; 1];
w = z(4:6)'*B/2;
dxdt = A*z(1:3);
dpdt = -A'*z(4:6);
%dzdt = z(13)*[dxdt; dpdt; 0];
%dzdt = [z(3); z(4); z(5); z(6); w(1); w(2); dpdt];
dzdt = [z(2); z(3); w; dpdt];
end
    