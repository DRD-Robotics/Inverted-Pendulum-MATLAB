function params = GetRodPendulumParams(wantDefault, Tfinal)
% get parameters for rob pendulum

% if default demo program values selected
if(wantDefault)
    
    % pendulum mass
    params.m = 0;
    
    % pendulum half length - length to CoG
    params.lh=0;
    
    % acceleration due to gravity
    params.g = 0;
    
    % viscous bearing friction
    params.mu = 0;
    
    % pendulum moment of inertia around CoG
    params.I = 0;
    
else
    % add real task pendulum values here
    % put in approriate values here
    params.m = 0.314;%Mass of pendulum
    params.lh=0.32; %length of rod =0.64. Half length = 0.32
    params.g = 9.8; %Gravity coeff
    params.mu = 0.05;% viscous bearing friction
    params.I = 0.043; % pendulum moment of inertia around CoG = 1/3 *(0.314)*(0.64)^2
    
end


