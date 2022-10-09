
%
% eas4710_new.m
% Rick Lind
%
% February 2020


%--------------------------------------------------------------------
%   Open-Loop Model
%--------------------------------------------------------------------
%      u         w         q        the        v         p         r        phi        x         y         z        psi    |  aileron     elevator    rudder
% AVL = [
%     -0.0484    0.2022   -0.8871   -9.8100    0.0000    0.0003   -0.0000    0.0000    0.0000   -0.0000    0.0000    0.0000     0.2444E-03  0.2515E-01  0.1221E-04
%     -0.2127   -8.4045   51.8752   -0.0000   -0.0000   -0.0168   -0.0003   -0.0000    0.0000   -0.0000    0.0000    0.0000     0.6744E-06  -1.212     -0.2499E-05
%      0.4021  -24.5876  -12.0937    0.0000   -0.0001   -0.0493   -0.0024    0.0000   -0.0000    0.0000   -0.0000    0.0000     0.2391E-03  -9.418     -0.5013E-04
%     -0.0000   -0.0000    1.0000    0.0000    0.0000   -0.0000   -0.0000    0.0000   -0.0000    0.0000   -0.0000    0.0000      0.000       0.000       0.000
%     -0.0000    0.0001    0.0000   -0.0000   -0.8582    0.8671  -54.6494    9.8100   -0.0000    0.0000   -0.0000   -0.0000     0.8187E-01 -0.7175E-06 -0.6950
%      0.0092   -0.5638   -0.1764   -0.0000   -5.3551  -23.7862    4.1617   -0.0000    0.0000   -0.0000    0.0000   -0.0000     -37.64     -0.8152E-01  -3.650
%      0.0001   -0.0068   -0.0018   -0.0000    4.8201    0.0199   -3.2127   -0.0000    0.0000   -0.0000    0.0000   -0.0000    -0.6924     -0.1049E-02   4.320
%     -0.0000   -0.0000    0.0000    0.0000    0.0000    1.0000    0.0000    0.0000   -0.0000    0.0000   -0.0000    0.0000      0.000       0.000       0.000
%      1.0000    0.0000   -0.0000    0.9049   -0.0000    0.0000    0.0000    0.0000    0.0000   -0.0000    0.0000    0.0000     -0.000      -0.000      -0.000
%      0.0000   -0.0000    0.0000   -0.0000    1.0000   -0.0000   -0.0000   -0.9049   -0.0000    0.0000   -0.0000   55.3325      0.000       0.000       0.000
%     -0.0000    1.0000   -0.0000  -55.3325    0.0000    0.0000    0.0000   -0.0000    0.0000   -0.0000    0.0000    0.0000     -0.000      -0.000      -0.000
%     -0.0000   -0.0000    0.0000    0.0000    0.0000   -0.0000    1.0000    0.0000   -0.0000    0.0000   -0.0000    0.0000      0.000       0.000       0.000];
%   

%       u         w         q        the        v         p         r        phi        x         y         z        psi    |  aileron     rudder      elevator  
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
% Check response to elevator to ensure states are logical
%
figure(1)                                    % open new figure window
u = step_tr(1,-deg2rad(10),.01,100);         % step of size 10 degrees starts at 1 second
y = trsp(P,u,100,.01,[0; 0; 0; 0; 0]);       % compute response for 10 seconds
%vplot(sel(y,[2 3 4],1));                     % plot angles and altitude
vplot(u,y)
legend('command','u','w','q','theta','h');          % label each signal
xlabel('Time (s)');                                 % label x-axis as time
ylabel('States');                                   % label y-axis as state magnitude


%--------------Notes from Response-------------------------------------%
%Negative elevator deflection creates an upwards elevation change along
%with a positive w value around 0.042 which seems surprisingly small
%if you convert it into alpha (alpha = w/u * 180). u becomes a negative
%value which is expected since it does take some forward velocity in order
%to climb
%----------------------------------------------------------------------%

%-----------------------------------Compute Kq
%
% Compute model with actuator and flight dynamics
%
systemnames = 'A P';                  % elements is P,A
inputvar = '[Kq]';                    % input is actually output from Kq
outputvar = '[P(3)]';                 % output is pitch rate
input_to_P = '[A]';                   % input to P is A 
input_to_A = '[Kq]';                  % input to A is Kq
sysoutname = 'S';                     % new system is called S
sysic                                 % make new system

%
% We need to insert a -1 into the loop to consider negative gains.
%   The problem is the rlocus commands only consider positive gains
%       such that a positive error generates a positive command.
%   If pitch-rate-command is greater than pitch-rate-state, 
%       then we want the elevator to generate a pitch-up moment
%       which means trailing-edge up which means negative error.
%   So, we want a positive error to generate a negative command
%       which means the controller gain must be negative.
%   An aircraft is different in that a positive error in pitch rate,
%   which means pitch-rate-command > pitch-rate-state, c
%
S = mmult(S,-1);

%
% Show root locus for Kq 
%
figure(2)                             % open new figure window
[a,b,c,d] = unpck(S);                 % extract a,b,c,d matrices
rlocus(a,b,c,d);                      % plot root locus
title('K_q Root Locus')
%
% Select gain that gives good damping
%
Kq = -0.59;

%
% Make closed-loop system with Kq
%
systemnames = 'A P Kq';               % system is P,A,Kq
inputvar = '[qc]';                    % input is pitch-rate command
outputvar = '[P(3)]';                 % output is pitch rate
input_to_P = '[A]';                   % input to P is A
input_to_A = '[Kq]';                  % input to A is Kq
input_to_Kq = '[qc-P(3)]';            % input to Kq is pitch error
sysoutname = 'T';                     % new system is called T
sysic                                 % make new system

%
% Compute response of closed-loop system
%
figure(3)
u = step_tr([1 2 3],[1 -1 0],.01,4); % Make doublet
y = trsp(T,u,4,.01);                 % compute response
vplot(u,y);                           % plot qc and q
legend('command','actual');           % label signals
xlabel('Time (s)');                   % xlabel is time
ylabel('Pitch Rate (deg/s');          % ylabel is pitch rate
title('Pitch Rate Doublet Response')

%----------------------------------------  Compute Kt

%
% Compute model with inner-loop Kq included
%
systemnames = '[A P Kq]';             % elements are P,A,Kq
inputvar = '[Kt]';                    % input is actually output from Kt
outputvar = '[P(4)]';                 % output is pitch angle
input_to_P = '[A]';                   % input to P is A
input_to_A = '[Kq]';                  % input to A is Kq
input_to_Kq = '[Kt-P(3)]';            % input to Kq is pitch error
sysoutname = 'S';                     % new system is called S
sysic                                 % make new system

%
% Show root locus for Kt
%
figure(4)                             % open new figure
[a,b,c,d] = unpck(S);                 % extract state-space matrices
rlocus(a,b,c,d);                      % plot root locus
title('K_t Root Locus')
%
% Select gain that gives good tracking
%
Kt = 12;                              % value of gain

% Make closed-loop system with Kt
%
systemnames = '[A P Kq Kt]';          % elements are P,A,Kq,Kt
inputvar = '[tc]';                    % input is pitch-angle command
outputvar = '[P(4)]';                 % output is pitch angle
input_to_P = '[A]';                   % input to P is A
input_to_A = '[Kq]';                  % input to A is Kq
input_to_Kq = '[Kt-P(3)]';            % input to Kq is pitch-rate error
input_to_Kt = '[tc-P(4)]';            % input to Kt is pitch-angle error
sysoutname = 'T';                     % new system is called T
sysic                                 % make new system

%
% Plot response to doublet
%
figure(5)                             % open new figure
u = step_tr([1 26 51],[10 -10 0],.01,60); % make doublet command
y = trsp(T,u,60,.01);                 % compute time response
vplot(u,y);                           % plot command and reponse
legend('command','actual');           % label signals
xlabel('Time (s)');                   % xlabel is time
ylabel('Pitch Angle (deg');           % ylabel is pitch angle
title('Pitch Angle Doublet Response')


%----------------------------------------  Compute Kh (Khh in simulink)
%
% Compute model for feeback to Kh
%
systemnames = '[A P Kq Kt]';          % elements are P,A,Kq,Kt
inputvar = '[Kh]';                    % input is actually output of Kh
outputvar = '[P(5)]';                 % output is altitude
input_to_P = '[A]';                   % input to P is A
input_to_A = '[Kq]';                  % input to A is Kq
input_to_Kq = '[Kt-P(3)]';            % input to Kq is pitch-rate error
input_to_Kt = '[Kh-P(4)]';            % input to Kt is pitch-angle error
sysoutname = 'T';                     % new system is called T
sysic                                 % make new system

%
% Compute root locus
%
figure(6)                             % open new figure
[a,b,c,d] = unpck(T);                 % extract state-space matrices
rlocus(a,b,c,d);                      % plot root locus
title('Khh Root Locus')

%
% Select gain that gives good damping and keeps stability
%
Kh = 12;                              % value of gain

%
% Make closed-loop system with Kt & Kh
%
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
% Plot response to step of 100 meters
%
figure(7)                             % open new figure
u = step_tr([1],[10],.01,20);        % make step command
y = trsp(T,u,20,.01);                 % compute time response
vplot(u,y);                           % plot command and response
%legend('command','u','alpha','q','theta','h','elevator');           % label signals
legend('Command','Response')
xlabel('Time (s)');                   % xlabel is time
ylabel('Altitude (m)');               % ylabel is altitude
title('Altitude Step Response')




