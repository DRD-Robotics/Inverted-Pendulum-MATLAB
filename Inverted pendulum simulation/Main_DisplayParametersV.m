% calculate Ardiuino simulation of inverted pendulum
% SFC of theta, thetaDot and position
% Lueberger observer only used to estimate theta, thetaDot
% estimate position made by directly integrating the velocity control signal
% with integral action on position
% nonlinear plant simulation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% all rights reserved
% Author: Dr. Ian Howard
% Associate Professor (Senior Lecturer) in Computational Neuroscience
% Centre for Robotics and Neural Systems
% Plymouth University
% A324 Portland Square
% PL4 8AA
% Plymouth, Devon, UK
% howardlab.com
% 23/01/2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

close all
clear all
clc

% use default parameters for demo program
wantDefault = 0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% get parameters for rod pendulum
params = GetRodPendulumParams(wantDefault, 5);
%Display in command window
params 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% get state space coefficients
c = GetStateSpaceCoesffs(wantDefault, params);
%Display in command window
c

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% get state space model with thetaDot, theta
%Display in command window
ssm = GetSSModel2x2V(wantDefault, params);
disp('ssm.A')
disp(ssm.A)
disp(' ')
disp('ssm.B')
disp(ssm.B)
disp(' ')
disp('ssm.C')
disp(ssm.C)
disp(' ')
disp('ssm.D')
disp(ssm.D)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% get state space model with thetaDot, theta and  position of cart
% integral action on POSition (changed to ssmPos for clarity)
ssmPos = GetSSModel4x4V(params);
disp('ssmPos.A')
disp(ssmPos.A)
disp(' ')
disp('ssmPos.B')
disp(ssmPos.B)
disp(' ')
disp('ssmPos.C')
disp(ssmPos.C)
disp(' ')
disp('ssmPos.D')
disp(ssmPos.D)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% build observer just for angle and angular velocity states
% calculate observer gain L here
%calculating SFC gain to set eigen values
PX=20*[-10 -11];
L = place(ssm.A, ssm.C', PX); %place func uses Ackermann formula to calculate pole placement 
disp(' ')
disp('Luenberger Gain Values')
disp(L)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% for state space model with thetaDot, theta and  position of cart
% calculate SFC gain K here
PK=8*[-1.1 -1.2 -0.01 -0.2];
K = place(ssmPos.A, ssmPos.B, PK);
disp(' ')
disp('State Feedback Controller Gain K')
disp(K)

