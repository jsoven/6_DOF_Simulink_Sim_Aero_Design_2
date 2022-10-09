
function y=eas4710_waypoint_yaw(x)


yaw_current = x(1);
yaw_command = x(2);

if norm(yaw_command-yaw_current) > 179
   if yaw_command < 0
        yaw_command = yaw_command + 360;
   else
        yaw_command = yaw_command - 360;
   end
end


y = yaw_command;



