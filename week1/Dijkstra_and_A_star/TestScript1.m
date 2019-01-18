%
% TestScript for Assignment 1
%
clear;
%% Define a small map
map = false(10);

% Add an obstacle
map (1:5, 6) = true;
map (7, 6) = true;
map (8, 1:2:6) = true;
map (1:4, 2:3) = true;
map ([7,9], 7:10) = true;

start_coords = [6, 2];
dest_coords  = [8, 9];

%%
close all;
algorithm = 'A_star';
% algorithm =  'Dijkstra';
if strcmp(algorithm, 'Dijkstra')
    fprintf('Dijkstra - algorithm\n')
    [route, numExpanded] = DijkstraGrid (map, start_coords, dest_coords);
elseif strcmp(algorithm, 'A_star')
    fprintf('A* - algorithm\n')
    [route, numExpanded] = AStarGrid (map, start_coords, dest_coords);
end

%HINT: With default start and destination coordinates defined above, numExpanded for Dijkstras should be 76, numExpanded for Astar should be 23.
