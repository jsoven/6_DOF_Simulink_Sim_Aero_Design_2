
%
% eas4710_lqr.m
%
% EAS 4710 : Aerospace Design II
% spring 2015
% Rick Lind
%


%
% This file will generate an altitude-hold controller
%

%
% Define open-loop dynamics with states of u,alpha,theta,q,h
%Feedback states of theta & q in degrees
%
AVL = [
    -0.0303    1.0913   -1.6554   -9.8100   -0.0000   -0.0000    0.0000    0.0000    0.0000   -0.0000    0.0000    0.0000    -0.2286E-08  0.2654E-08  0.1405E-02
    -0.8341   -5.2075   11.0330   -0.0000    0.0000    0.0000    0.0000   -0.0000    0.0000   -0.0000    0.0000    0.0000     0.3698E-08 -0.4352E-08 -0.6689E-01
     0.1049   -0.7977   -2.6107    0.0000    0.0000    0.0000   -0.0000    0.0000   -0.0000    0.0000   -0.0000    0.0000    -0.1046E-07 -0.3419E-07 -0.4297    
    -0.0000   -0.0000    1.0000    0.0000    0.0000   -0.0000   -0.0000   -0.0000   -0.0000    0.0000   -0.0000    0.0000      0.000       0.000       0.000    
     0.0000   -0.0000   -0.0000   -0.0000   -0.0596    1.8396  -12.3643    9.8100   -0.0000    0.0000   -0.0000   -0.0000     0.6493E-03 -0.4860E-02  0.3007E-10
     0.0000   -0.0000   -0.0000   -0.0000   -6.8996  -22.7061    5.5074   -0.0000    0.0000   -0.0000    0.0000   -0.0000    -0.4853     -0.3314E-01 -0.3697E-08
    -0.0000    0.0000    0.0000   -0.0000    0.3159   -1.7697   -0.1907   -0.0000    0.0000   -0.0000    0.0000   -0.0000    -0.1007E-01  0.4141E-01 -0.3976E-09
    -0.0000   -0.0000    0.0000    0.0000    0.0000    1.0000    0.0000    0.0000   -0.0000    0.0000   -0.0000    0.0000      0.000       0.000       0.000    
     1.0000    0.0000   -0.0000    1.6300   -0.0000    0.0000    0.0000    0.0000    0.0000   -0.0000    0.0000    0.0000     -0.000      -0.000      -0.000    
     0.0000   -0.0000    0.0000   -0.0000    1.0000   -0.0000   -0.0000   -1.6300   -0.0000    0.0000   -0.0000   12.3933      0.000       0.000       0.000    
    -0.0000    1.0000   -0.0000  -12.3933    0.0000    0.0000    0.0000    0.0000    0.0000   -0.0000    0.0000    0.0000     -0.000      -0.000      -0.000    
    -0.0000   -0.0000    0.0000    0.0000    0.0000   -0.0000    1.0000    0.0000   -0.0000    0.0000   -0.0000    0.0000      0.000       0.000       0.000
];
%
% Extract longitudinal components
%    using states of u      (m/s)
%                    w      (m/s)
%                    q      (rad/s)
%                    theta  (rad)
%                    z      (m)
%     
%
%   using inputs of  de     (rad)
%                    
%
%   using outputs of u      (m/s)
%                    w      (m/s)
%                    q      (deg/s)
%                    theta  (deg)
%                    h      (m)
%
%
A = AVL([1 2 3 4 11],[1 2 3 4 11]);
B = AVL([1 2 3 4 11],[15]);
C = diag([1 1 57 57 -1]);
D = zeros(5,1);
P = pck(A,B,C,D);  %Aircraft longitudinal is defined as P


% Define actuator dynamic
%
A = nd2sys([20],[1 20]);              % actuator is called A
[Aele,Bele,Cele,Dele] = tf2ss([20],[1 20]);
A = pck(Aele,Bele,Cele,Dele);

%
% Make closed-loop system with Kt & Kh
%
Kt = 12;
Kh = 12;
Kq = -0.59;
systemnames = '[A P Kq Kt Kh]';       % elements are P,A,Kq,Kt,Kh
inputvar = '[hc]';                    % input is altitude command
outputvar = '[P(5)]';                    % output is all states followed by control
input_to_P = '[A]';                   % input to P is A
input_to_A = '[Kq]';                  % input to A is Kq
input_to_Kq = '[Kt-P(3)]';            % input to Kq is pitch-rate error
input_to_Kt = '[Kh-P(4)]';            % input to Kt is pitch-angle error
input_to_Kh = '[hc-P(5)]';            % input to Kh is altitude error
sysoutname = 'T';                     % new system is called T
sysic                                 % make new system

%
% Plot response to step of 1 meters
%
figure(1)                            % open new figure
u = step_tr([0],[1],.01,30);        % make step command
y = trsp(T,u,30,.01);                 % compute time response
vplot(u,y);                           % plot command and response
%legend('command','u','alpha','q','theta','h','elevator');           % label signals
legend('Command','Response')
xlabel('Time (s)');                   % xlabel is time
ylabel('Altitude (m)');               % ylabel is altitude
title('Altitude Step Response')

%-------------------------------------------------------------
%    DESIGN LQR CONTROLLER
%-------------------------------------------------------------

%
%
% Extract the state-space matrices
%
[A,B,C,D] = unpck(P);

%
% Rearrange states so altitude is the first state
%
v = [5 1 2 3 4];
A = A(v,v);
B = B(v,:);

%
% Use identity as output matrix so all states are used in feedback
%
C = eye(5);
D = zeros(5,1);

%
% Assemble into state-space system
%
P = ss(A,B,C,D);

%
% Define the weighting matrices to emphasize altitude tracking
%
Q = .01*eye(5);
Q(1,1) = 1000;
R = 1;

%
% Compute the optimal controller
%
K = lqr(A,B,Q,R);

%
% Separate the feedback and feedforward
%
k = K(1);

%
% Make the closed-loop block diagram
%
T = feedback(P,K);
T = series(k,T);

%Create data to plot PID
t = y(1:end-1,2);
y = y(1:end-1,1);
figure(2)
[yy,tt] = step(T,30);
plot(t,y,tt,yy(:,1));
legend('PID','LQR','Location','southeast');
axis([0 30 0 1.2]);
xlabel('Time (s)');
ylabel('Altitude (m)');

plot(tt,yy)


