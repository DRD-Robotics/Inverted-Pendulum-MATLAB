function ssm = GetSSModel4x4V(params)
% get 4x4 state space model with thetaDot, theta and position of cart as state variables
% integral action


% add real task pendulum values here
% .....
% compute values
c = GetStateSpaceCoesffs(false, params);
ssm.A = [0 1 0 0; -c.a2 -c.a1 0 0; 0 0 0 0; 0 0 1 0];   
ssm.B = [c.b0; -c.a1*c.b0; 1; 0];   
ssm.C = [1 0 0 0];
ssm.D = [0];

