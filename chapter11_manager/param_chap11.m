
% need to add the following to your parameter file:
 
% chapter 11 - path manager
% number of waypoints in data structure
P.size_waypoint_array = 100;
P.R_min = P.Va0^2/P.gravity/tan(P.phi_max);

% create random city map
city_width      = 2000;  % the city is of size (width)x(width)
building_height = 300;   % maximum height of buildings
%building_height = 1;   % maximum height of buildings (for camera)
num_blocks      = 5;    % number of blocks in city
street_width    = .8;   % percent of block that is street.
P.pd0           = -h0;  % initial height of MAV
P.map = createWorld(city_width, building_height, num_blocks, street_width);

