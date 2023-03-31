function c = GetStateSpaceCoesffs(wantDefault, params)
% get state space coefficients

% if default demo program values selected
if(wantDefault)
    % dummy values set here are for demo only
    c.b0 =  3;
    c.b1 = 0;
    c.a0 = 1;
    c.a1 = 1;
    c.a2 = -25;
else
  
    % add real task pendulum values here
    % .....


    % compute appropiate values here
	
	% equation (13)
    c.b0 =  params.m * params.lh / (params.I + params.m * params.lh^2); 
    c.b1 = 0; %n/a
    c.a0 = 0; %n/a
	
	% equation (11)
    c.a1 = params.mu / (params.I + params.m * params.lh^2);
	
	% equation (12)
    c.a2 = -params.m * params.g * params.lh / (params.I + params.m * params.lh^2);
end
