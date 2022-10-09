
%------------------------------------------------------------------------
% Mission data
%------------------------------------------------------------------------

%
% Define the waypoints that are available to all subroutines
%
global waypoint;	% counter 
global y1;		    % waypoint 1
global y2;		    % waypoint 2
global y3;		    % waypoint 3

%
% Set the values
%
waypoint = 1;
y1 = [-100;-50;10];
y2 = [50;50;15];
y3 = [250;-25;25];
% y1 = [-100;50; 20];
% y2 = [-250;50;170];
% y3 = [-300;-250;50];

%------------------------------------------------------------------------
% Vehicle dynamic
%------------------------------------------------------------------------


%
% Set trim condition
%
V = 12; 		% velocity at trim
H = 0;			% altitude at trim

%Create full State-Space Matrix
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
% using states of    u      (m/s)
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
Along = AVL([1 2 3 4 11],[1 2 3 4 11]);
Blong = AVL([1 2 3 4 11],[15]);
Clong = diag([1 1 57 57 -1]);
Dlong = zeros(5,1);


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
Alatd = AVL([5 6 7 8 12 10],[5 6 7 8 12 10]);
Blatd = AVL([5 6 7 8 12 10],[13 14]);
Clatd = diag([1 57 57 57 57 1]);
Dlatd = zeros(6,2);

%
% Define longitudinal actuator
%
%  Aele = -20;
%  Bele = 20;
%  Cele = -1;
%  Dele = 0;
[Aele,Bele,Cele,Dele] = tf2ss([100],[1 100]);

%
% Define lateral-directional actuators
%
Aact = [-20.2 0;0 -20.2];
Bact = [20.2 0;0 20.2];
Cact = [1 0;0 1];
Dact = [0 0;0 0];

%------------------------------------------------------------------------
% Controller
%------------------------------------------------------------------------

%
% Low-pass filter on altitude command
%
%[Aalt,Balt,Calt,Dalt] = tf2ss([1 .3],[3 7.2]);


%
% Washout filter on yaw rate
%
Atau = -1;
Btau = 1;
Ctau = -1;
Dtau = 1;

%
% Gains
%
Kd =  .1;
%Kd =  2.6;     	% derivative gain on heading error
Kh =  1 ;
%Kh =  5;       % proportional gain on altitude error  (done) unmodified/
Kk = 0.01;       % proportional gain on roll error

Kq = 12;
%Kq = 2;         % proportional gain on pitch rate     (done) questionable (from alt hold)
Kt = -.3;        % proportional gain on pitch error    (done) less questionable (from alt hold)
%Kt = -5;
Kp = -88 ;	    % proportional gain on roll rate      (done high value because large overshoots without)
%Kp = -.7;
Kr = .01;	    % proportional gain on yaw rate       (done turned out to be negative, didn't need to modify much)
Khh = 1;

%Current Gain Values
Kd = 0;      %Corrected
Kh = 0.25;     %Corrected
Kk = -1.2;  %Corrected
Kr = 0.85;   %Corrected
Kp = 0.005;  %Corrected
Kq = -0.59;
Kt = 12;
Khh = 12;

% %For Testing Longitudianl Stability
% Kd = 0;      %Corrected
% Kh = 0;     %Corrected
% Kk = 0;  %Corrected
% Kr = 0;   %Corrected
% Kp = 0;  %Corrected
% Kq = -0.2;
% Kt = 15;
% Khh = 1.5;
%------------------------------------------------------------------------
% Attempted solution using SS models
%------------------------------------------------------------------------
% P = pck(Along,Blong,Clong,Dlong);
% A = pck(Aele,Bele,Cele,Dele);
% systemnames = ['A P Kq Kt Khh'];
% inputvar = '[hc]';                    % input is altitude command
% outputvar = '[P]';                 % output is altitude
% input_to_P = '[A]';                   % input to P is A
% input_to_A = '[Kq]';                  % input to A is Kq
% input_to_Kq = '[Kt-P(5)]';            % input to Kq is pitch-rate error
% input_to_Kt = '[Khh-P(4)]';            % input to Kt is pitch-angle error
% input_to_Kh = '[hc-P(1)]';            % input to Kh is altitude error
% sysoutname = 'T';                     % new system is called T
% sysic                                 % make new system
% 
% %
% % Plot response to step of 100 meters
% %
% figure                                % open new figure
% u = step_tr([1],[100],.0001,30);        % make step command
% y = trsp(T,u,30,.01);                 % compute time response
% vplot(u,y);                           % plot command and response
% legend('command','h','u','alpha','theta','q');           % label signals
% xlabel('Time (s)');                   % xlabel is time
% ylabel('Altitude (m)');               % ylabel is altitude


%------------------------------------------------------------------------
% Simulation
%------------------------------------------------------------------------

%
% Run the simulation
%
[t,x,y] = sim('eas4710_waypoint_block_modified',[0 80]);

%------------------------------------------------------------------------
% Plots
%------------------------------------------------------------------------

%
% Plot the North-East position
%
figure(1)
plot(y(:,2),y(:,1),'k',y1(2),y1(1),'ro',y2(2),y2(1),'ro',y3(2),y3(1),'ro');
ylabel('East');
xlabel('North');

%
% Plot the North-Altitude position
%
figure(2)
plot(y(:,2),y(:,11),'k',y1(2),y1(3),'ro',y2(2),y2(3),'ro',y3(2),y3(3),'ro');
ylabel('Altitude');
xlabel('North');


%
% Plot the Altitude Command/Actual
%
figure(3)
plot(t,y(:,11),'r',t,y(:,10));
ylabel('Altitude');
xlabel('Time (s)');
legend('Actual','Command');

%
% Plot the Heading Command/Actual
%
figure(4)
plot(t,y(:,3),'r',t,y(:,4),'k')
ylabel('Heading (deg)');
xlabel('Time (s)');
legend('Actual','Command');

%
% Plot the Roll Command/Actual
%
figure(5)
plot(t,y(:,5),'r',t,y(:,6),'k');
ylabel('Roll Angle (deg)');
xlabel('Time (s)');
legend('Actual','Command');

%
% Plot the control surfaces
%
figure(6)
plot(t,y(:,7),'k',t,y(:,8),'r',t,y(:,9),'b');
ylabel('Deflection (deg)');
xlabel('Time (s)');
legend('Aileron','Rudder','Elevator');

%
% Plot the 3-D trajectory 
%
figure(7)
h=plot3(y(:,2),y(:,1),y(:,11),'k', ...
		y1(2),y1(1),y1(3),'ro', ...
		y2(2),y2(1),y2(3),'ro', ...
		y3(2),y3(1),y3(3),'ro');
zlabel('Altitude');
ylabel('East');
xlabel('North');


%
% Plot altitude hold state-space outputs
%
% figure(8)
% plot(t,y(:,13),t,y(:,14),t,y(:,15),t,y(:,16),t,y(:,17))
% xlabel('Time')
% legend('h','u','alpha','theta','q')




