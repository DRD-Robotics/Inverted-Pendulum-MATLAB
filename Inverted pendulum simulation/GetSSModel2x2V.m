function ssm = GetSSModel2x2V(wantDefault, params)
% get 2x2 state space model with thetaDot, theta as state variables

% if default demo program values selected
if(wantDefault)
    
    % default values
    % for velocity control
    % % get inverted configuation
    ssm.A = [];     % we dont use A yet but you will need it later
    ssm.B = [];     % we dont use B yet but you will need it later
    ssm.C = [1 0;];
    ssm.D = 0;
    ssm.K = [];     % we dont use K yet but you will need it later
    
else
    % calculate appropiate values here
	% values a1, a2 & b0 taken from GetStateSpaceCoesffs.m 
    c = GetStateSpaceCoesffs(wantDefault, params);
    ssm.A = [0 1; -c.a2 -c.a1];   
    ssm.B = [c.b0; -c.a1*c.b0];   
    ssm.C = [1 0];
    ssm.D = [0]; % No D
end
