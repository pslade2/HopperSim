%% settings
plot = 0; %0 to not plot
fast = 0; %1 to plot faster response (doesn't meet control constraint)
discrete = 1;

%% designing body controller
s = tf('s');

Ib = 1; %intertia of the body, affects the overall gain of combined controller
G = 1/(Ib*s^2); %plant double integrator

%desired placement of poles
if(fast)
    re = -18;%-1;
    im = 18;%2;
    pole = -40; 
else
    re = -1;
    im = 2;
    pole = -10; 
end
sd = re+im*j;
%angle(evalfr(G,sd))
cur_phase = angle(evalfr(G,sd));
phase_add = pi - cur_phase;
%choose arbitrary pole placement

%pole = -10; %for slow
zero = -2/tan(phase_add + atan(2/-pole))
K_gain = norm(sd^2)*norm(sd-pole)/norm(sd-zero);

if(zero>0)
    D = K_gain*(s+zero)/(s-pole)
else
    D = K_gain*(s-zero)/(s-pole)
end

%% discrete controller
if(discrete)
    T = 0.001; % 1ms sample time -- 1000hz
    z = tf('z',T)
    zd = exp(sd*T)
    Gz = c2d(G,T,'zoh')
    phid_add = angle(evalfr(Gz,zd))*180/(2*pi)
    poled = -0.8;
    %zerod = 
    
    %Kz = 1/(
    %rltool(Gz)
    
    
    %compare continuous and discrete controllers
    %margin(G*D)
    
end

%% plots
if(plot)
    figure(1);
    bode(G*D) %check bandwidth and magnitude
    
    CL = G*D/(1+G*D);
    figure(2);
    stepplot(CL)
    
    rltool(G*D)%RL is stable
    
    bw = bandwidth(CL);
end                                    