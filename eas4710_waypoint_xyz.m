
function V=eas4710_waypoint_xyz(x)

%
% The order of inputs must be
%
% x(1) is delta_u
% x(2) is beta (degrees)
% x(3) is alpha (degrees)
% x(4) is delta_phi (degrees)
% x(5) is delta_theta (degrees)
% x(6) is delta_psi (degrees)
% x(7) is trim velocity


%
% Convert the angles from degrees into radians
%
x(2) = x(2)*pi/180;
x(3) = x(3)*pi/180;
x(4) = x(4)*pi/180;
x(5) = x(5)*pi/180;
x(6) = x(6)*pi/180;

%
% Convert from alpha,beta into velocities
%
x(2) = x(2)*x(7);
x(3) = x(3)*x(7);

%
% Rename variables
%
u = x(1)+x(7);
v = x(2);
w = x(3);
phi = x(4);
theta = x(5);
psi = x(6);

%
% Compute the nonlinear velocity for North
%
Xdot = u*cos(theta)*cos(psi) ...
	+ v*(sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi)) ...
	+ w*(cos(phi)*sin(theta)*cos(psi) - sin(phi)*sin(psi));

Ydot = u*cos(theta)*sin(psi) ...
	+ v*(sin(phi)*sin(theta)*sin(psi) + cos(phi)*cos(psi)) ...
	+ w*(cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi));

Zdot = -u*sin(theta) ...
	+ v*sin(phi)*cos(theta) ...
	+ w*cos(phi)*cos(theta);


%
% Output the velocities
%
V = [Xdot;Ydot;Zdot];

