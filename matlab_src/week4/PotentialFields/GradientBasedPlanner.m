function route = GradientBasedPlanner (f, start_coords, end_coords, max_its)
% GradientBasedPlanner : This function plans a path through a 2D
% environment from a start to a destination based on the gradient of the
% function f which is passed in as a 2D array. The two arguments
% start_coords and end_coords denote the coordinates of the start and end
% positions respectively in the array while max_its indicates an upper
% bound on the number of iterations that the system can use before giving
% up.
% The output, route, is an array with 2 columns and n rows where the rows
% correspond to the coordinates of the robot as it moves along the route.
% The first column corresponds to the x coordinate and the second to the y coordinate

[gx, gy] = gradient (-f);
%%% All of your code should be between the two lines of stars.
% *******************************************************************

route = [start_coords];
for i=1:max_its
    current_point = route(end,:);
     if sum( abs(current_point-end_coords) ) < 5.0
         disp('Reached the goal !');
         break
     end
    ix = round( current_point(2) ); % X and Y axis are swaped
    iy = round( current_point(1) );
    w = 10;
    vx = mean( mean( gx(ix-w/2:ix+w/2, iy-w/2:iy+w/2) ) );
    vy = mean( mean( gy(ix-w/2:ix+w/2, iy-w/2:iy+w/2) ) );
    dt = 1 / norm([vx, vy]);
    next_point = current_point + dt*[vx, vy];
    route = vertcat(route, next_point);
end

% figure
% plot (route(:,1), route(:,2), 'r', 'LineWidth', 2);
% grid on

% *******************************************************************
end
