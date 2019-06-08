function [link_time, link_speed] = getSpeed(num_vehicles, capacity, free_flow_speed, link_length)


% BPR function parameters
alpha = 0.15;
beta = 4;

% link length in meters
% free_flow_speed in meters/second
free_flow_time = link_length/free_flow_speed;

% link_time = free_flow_time*(1+alpha*(num_vehicles/capacity)^beta);
link_time = fzero(@(x) full(free_flow_time)*(1+alpha*(full(num_vehicles)*full(free_flow_time)/(full(capacity)*x))^beta) - x, full(free_flow_time));
link_speed = link_length/link_time;

end
