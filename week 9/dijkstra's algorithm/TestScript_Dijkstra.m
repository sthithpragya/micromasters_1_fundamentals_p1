%
% TestScript for Assignment Week 9
%

%% Define a small map
map = false(3);

% Add an obstacle
% map (1, 3) = true;

start_coords = [2, 1];
dest_coords  = [2, 3];

%%
[route, numExpanded] = DijkstraGrid(map, start_coords, dest_coords, true)
