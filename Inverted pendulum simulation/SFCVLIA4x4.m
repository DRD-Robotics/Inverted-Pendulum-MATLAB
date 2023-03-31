function [xhatout, tout, xout, uout] = SFCVLIA4x4(VCPendDotCB, a1, a2, b0, ...
                             A, B, C, D, Ahat, Bhat, K, L, t, x0, maxSpeed)
% nonlinear simulate state space model using C-language compatible formuation
          %%%%VARIABLES%%%%%
% performs integration by Euler's method
% VCPendDotCB is the callback function to compute xDot
% a1, a2, b0 are coedfficients  
% A,B,C&D are the state space model output matrices
% K is the state feedback control gain
% L is Luenburger observer gain
% Ahat estimated state from model (in the closed loop, these are "copys/estimates" of A)
% Bhat estimated state from model (in the closed loop, these are "copys/estimates" of B)
% t is a vector of time samples
% x0 is the initial state vector
% Max speed = max speed
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% get signal length
len = length(t) -1;

% initiate outputs
xout = zeros(4,len);
xhatout = zeros(2,len); % The estimated states by the observer 
tout = zeros(1,len);
uout = zeros(1,len);

% record the initial state
xhat = [0; 0];
xhatout(:,1) = xhat; %1st colomn in array (is zero?)

x = x0;
xout(:, 1) = x0;     %1st colomn in array (is zero?)

% for all remaining data points, simulate state-space model using C-language compatible formulation
for idx = 1:len
    
    % record time
    tout(idx) = t(idx);

    % get the duration between updates
    h = t(idx+1) - t(idx);
    
    % Observer estimated state feedback rule to compute variable u
    u = -( K(1)*xhat(1) + K(2)*xhat(2)-K(3)*x(3) ) -K(4)*x(4);
    u = max(min(u, maxSpeed), -maxSpeed);
    
    % calculate state derivative from non-linear pendulum equations
    xDot = VCPendDotCB(a1, a2, b0, x, u);
    
  
    
    % update the state using Euler integration over time step h  
    % update the remaining simulated state X using Euler integration over
    % time step h
    x(1) = x(1) + h * xDot(1);
    x(2) = x(2) + h * xDot(2);
    x(3) = x(3) + h * u;
    x(4) = x(4) + h * x(3);
    
    y = C*x;
    
    % update the observed states using Euler integration c-lang style
    xhatDot = Ahat*xhat + Bhat*u + L'*y; %L'*y = y_correction term 
    %observer state update
    xhat(1) = xhat(1) + h * xhatDot(1);
    xhat(2) = xhat(2) + h * xhatDot(2);
   
    % record the state
    uout(:,idx)= u;
    xout(:,idx) = x;
    xhatout(:,idx) = xhat;
end
