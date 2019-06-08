function [final_route] = findFinalPath(route, origin_node, dest_node, NodesLocation, LinkTime)
	% converts a precomputed path from station to station to a o-d path by keeping
	% the majority of the path intact

	%% for the origin
	% find closest node in path
	closest_path_ind = dsearchn(NodesLocation(route, :), NodesLocation(origin_node,:));
	% find shortest path to that node from origin
	init_route = findRoute(origin_node, route(closest_path_ind), LinkTime);
	% update route
    
	final_route = [init_route route(closest_path_ind + 1 : end)'];

	%% for the destination
	% find closest node in path
	closest_path_ind = dsearchn(NodesLocation(final_route, :), NodesLocation(dest_node,:));
	% find shortest path to that node from origin
	end_route = findRoute(final_route(closest_path_ind), dest_node, LinkTime);
	% update route
	final_route = [final_route(1:closest_path_ind) end_route(2:end)];

end