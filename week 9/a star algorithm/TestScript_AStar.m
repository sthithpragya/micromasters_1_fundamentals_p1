%
% TestScript for Assignment Week 9
%

%% Define a small map
map = false(5);

% Add an obstacle
map (1:5, 3) = true;

start_coords = [1, 1];
dest_coords  = [4, 5];

%%
[route, numExpanded] = AStarGrid2(map, start_coords, dest_coords, true)
