
function y=eas4710_waypoint_set(x)

global waypoint;
global y1;
global y2;
global y3;

if waypoint == 1

    y = y1;
    if norm(x-y1) < 20
        waypoint = 2;
        y = y2;
     end
end

if waypoint == 2
    y = y2;

    if norm(x-y2) < 20
        waypoint = 3;
        y = y3;
     end
end

if waypoint == 3
   y=y3;
end


