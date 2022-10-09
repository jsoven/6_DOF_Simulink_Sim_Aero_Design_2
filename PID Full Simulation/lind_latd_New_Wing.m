

%
% System matrices from AVL
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
% Extract lateral-directional components
%    using states of v      (m/s)
%                    p      (rad/s)
%                    r      (rad/s)
%                    phi    (rad)
%                    psi    (rad)
%                    y      (m)
%
%   using inputs of  da     (rad)
%                    dr     (rad)
%
%   using outputs of v      (m/s)
%                    p      (deg/s)
%                    r      (deg/s)
%                    phi    (deg)
%                    psi    (deg)
%                    y      (m)
%
A = AVL([5 6 7 8 12 10],[5 6 7 8 12 10]);
B = AVL([5 6 7 8 12 10],[13 14]);
C = diag([1 57 57 57 57 1]);
D = zeros(6,2);
P = pck(A,B,C,D);


%
% actuator
%
A1 = nd2sys(20.2,[1 20.2]);
A2 = nd2sys(20.2,[1 20.2]);
A = daug(A1,A2);



% %Check to make sure responses seem accurate
% %
% % make single input system
% systemnames = 'A P';
% inputvar = '[u]';
% input_to_A = '[0*u;u]';
% input_to_P = '[A]';
% outputvar = '[P(1:5)]';
% sysoutname = 'S';
% sysic
% 
% u = step_tr([1 2 3],[0.1745*2 -0.1745*2 0],.01,3); % Make doublet for
% %aileron
% %u = step_tr([1 5],[0.1745 0],.01,8); %Make single step for rudder
% y = trsp(S,u,10,.01);                   % compute response
% vplot(u,y);                           % plot qc and q
% legend('command','v','p','r','phi','psi','y');           % label signals
% %legend('Command','phi (roll angle)')
% xlabel('Time (s)');                   % xlabel is time
% ylabel('Roll Angle (deg');          % ylabel is pitch rate
% 


%-------------------------------------------------------------------------%
%This plot tells us a few things. It says that for an input roll deflection
%(in radians), after 1 second, an equal sized roll angle will be
%experienced i.e. for a 10 degree deflection angle (input in radians
%though) there will be a 10 degree roll angle after 1 second. Additionally
%this model has a positive aileron deflection of left aileron up so a
%positive roll (assuming positive roll is measured from the y-axis to
%z-axis as positive which would follow right hand rule) would require a
%negative aileron input.
%Additionally, a left roll creates a negative yaw angle which would be a
%nose left movement which does not align with what is to be expected

%For a positive rudder command,a positive yaw command is induced which
%shouldn't be that way. A postive rudder command moves the rudder surface
%to the left which induces a nose left moment. Either way we're just going
%to run with it and maybe check later. So what happens is a positve rudder
%command creates a nose right movment.

%    SUMMARY
%Positive aileron = roll left
%Positive rudder  = nose right
%-------------------------------------------------------------------------%


%------------------------------------------------------------------
% Make the yaw damper
%------------------------------------------------------------------

%
% Define the washout filter
%
W = nd2sys([1 0],[1 1]);

%
% Make the open-loop block diagram
%
systemnames = 'A P W';
inputvar = '[Kr]';
outputvar = '[W]';
input_to_P = '[A]';
input_to_A = '[0*Kr;Kr]';
input_to_W = '[P(3)]';
sysoutname = 'S';
sysic
 

%Show root locus for Kr

figure
[a,b,c,d] = unpck(S);
rlocus(a,b,c,d);
title('K_r Root Locus')


%
% Pick a gain that gives good damping
%
Kr = 0.85;



%Create Closed loop yaw tracking
%
systemnames = 'A P W Kr';
inputvar = '[psi]';
outputvar = '[P(3)]';
input_to_P = '[A]';
input_to_A = '[0*psi;psi-Kr]';
input_to_W = '[P(3)]';
input_to_Kr = '[W]';
sysoutname = 'S';
sysic


figure
u = step_tr([1 1.2 1.4],[1 -1 0],.01,10); % Make doublet
y = trsp(S,u,10,.01);                  % compute response
vplot(u,y);                           % plot qc and q
legend('command','actual');           % label signals
xlabel('Time (s)');                   % xlabel is time
ylabel('Yaw Rate (deg');              % ylabel is yaw rate

%--------------------------------------------
% Directly track roll with yaw damper
%
%
% Make the open-loop block diagram
%ROLL FEEDBACK AND INPUT COULD BE INCORRECT SIGNS HERE
%*****************************************************
%
systemnames = 'A P W Kr';
inputvar = '[Ki]';
outputvar = '[-P(4)]';
input_to_P = '[A]';
input_to_A = '[Ki;-Kr]';
input_to_W = '[P(3)]';
input_to_Kr = '[W]';
sysoutname = 'S';
sysic

figure
[a,b,c,d] = unpck(S);
rlocus(a,b,c,d);
title('K_k Root Locus')


%Create closed loop roll controller with yaw damper
Ki = -1.2;
systemnames = 'A P W Kr Ki';
inputvar = '[phi]';
outputvar = '[P(4)]';
input_to_P = '[A]';
input_to_A = '[Ki;-Kr]';
input_to_W = '[P(3)]';
input_to_Kr = '[W]';
input_to_Ki = '[phi-P(4)]';
sysoutname = 'S';
sysic

%Check Roll Response
figure
u = step_tr([1 1.2 1.4],[1 -1 0],.01,3); % Make doublet
y = trsp(S,u,3,.01);                 % compute response
vplot(u,y);                           % plot qc and q
legend('command','actual');           % label signals
xlabel('Time (s)');                   % xlabel is time
ylabel('Roll Angle (rad)');          % ylabel is pitch rate
legend('Command','psi')
%legend('command','v','p','r','phi','psi','y');  

return
%-------------------------------------------------------------------------%
%NOTES FROM ROLL RESPONSE
%Current setup is for a value to come into the desired angle and be
%converted to the opposite value (negative aileron = positive roll) using a
%-Ki value (Kk on Simulink). Feedback is then sent back to the original
%command where the original command should be in the direction of desired
%roll angle and the output roll angle should also be in the desired angle
%-------------------------------------------------------------------------%




% make heading tracker
systemnames = 'A P W Kr Ki';
inputvar = '[psi]';
outputvar = '[P(5)]';
input_to_P = '[A]';
input_to_A = '[Ki;-Kr]';
input_to_W = '[P(3)]';
input_to_Kr = '[W]';
input_to_Ki = '[psi-P(4)]';
sysoutname = 'S';
sysic

figure
[a,b,c,d] = unpck(S);
rlocus(a,b,c,d)
title('K_h Root Locus')





%Compute Heading command Response
Ky = 0.25;

systemnames = 'A P W Kr Ki Ky';
inputvar = '[psi]';
outputvar = '[P(5)]';
input_to_P = '[A]';
input_to_A = '[Ki;-Kr]';
input_to_W = '[P(3)]';
input_to_Kr = '[W]';
input_to_Ki = '[Ky-P(4)]';
input_to_Ky = '[psi-P(5)]';
sysoutname = 'S';
sysic

figure
u = step_tr([1 3 225],[0 153 0],.01,10); % Make doublet
y = trsp(S,u,250,.01);                 % compute response
vplot(u,y);                           % plot qc and q
legend('Command','Yaw Angle');           % label signals
xlabel('Time (s)');                   % xlabel is time
ylabel('Magnitude (Deg)');          % ylabel is pitch rate


%Repeat Response but now get output of Deflections

systemnames = 'A P W Kr Ki Ky';
inputvar = '[psi]';
outputvar = '[A]';
input_to_P = '[A]';
input_to_A = '[Ki;-Kr]';
input_to_W = '[P(3)]';
input_to_Kr = '[W]';
input_to_Ki = '[Ky-P(4)]';
input_to_Ky = '[psi-P(5)]';
sysoutname = 'S';
sysic

figure
u = step_tr([1 3 5],[1 -1 0],.01,10); % Make doublet
y = trsp(S,u,10,.01);                 % compute response
vplot(y);                             % plot qc and q
legend('Aileron Command','Rudder Command');           % label signals
xlabel('Time (s)');                        % xlabel is time
ylabel('Deflection Angle (rad)');          % ylabel is pitch rate

%------------------------------------------------------------------
% Make the roll-rate tracker
%------------------------------------------------------------------

%
% Make the open-loop block diagram
%
systemnames = 'A P W Kr';
inputvar = '[Kp]';
outputvar = '[P(2)]';
input_to_P = '[A]';
input_to_A = '[-Kp;-Kr]';
input_to_W = '[P(3)]';
input_to_Kr = '[W]';
sysoutname = 'S';
sysic

%
% Show root locus for Kp
%
figure
[a,b,c,d] = unpck(S);
rlocus(a,b,c,d);
title('K_p Root Locus')


%
% Nothing looks great
%
Kp = 0.005;


%
% Make closed-loop system with Kp
%
systemnames = 'A P W Kr Kp';
inputvar = '[p]';
outputvar = '[P(2)]';
input_to_P = '[A]';
input_to_A = '[p-Kp;-Kr]';
input_to_W = '[P(3)]';
input_to_Kr = '[W]';
input_to_Kp = '[P(2)]';
sysoutname = 'S';
sysic

%
% Compute response of closed-loop system
%
figure
u = step_tr([1 1.2 1.4],[1 -1 0],.01,3); % Make doublet
y = trsp(S,u,3,.01);                 % compute response
vplot(u,y);                           % plot qc and q
legend('command','actual');           % label signals
xlabel('Time (s)');                   % xlabel is time
ylabel('Roll Rate (deg/s');          % ylabel is pitch rate


