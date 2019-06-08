function [Route] = FreeRouteAstar(start,goal,RoadGraph,RoadCost,heuristic_est)

N=length(RoadGraph);
if nargin<=4
    heuristic_est = zeros(N); %revert to Dijkstra
end

%forked from the AMoD-congestion implementation

ClosedSet = [];
OpenSet = [start];




% % Heuristic, specific to this square grid implementation
% rN=sqrt(N);
% FindNode=@(i,j) rN*(i-1)+j;
% FindX=@(node) mod(node-1,rN)+1;
% FindY=@(node) ceil(node/rN);
% persistent heuristic_est
% heuristic_est=zeros(N,N);
% for ii=1:N
%     for jj=1:N
%         heuristic_est(ii,jj) = abs(FindX(jj)-FindX(ii)) + abs(FindY(jj)-FindY(ii));
%     end
% end
% % /heuristic

CameFrom=-1*ones(N);
g_score = Inf*ones(N,1);
f_score = Inf*ones(N,1);
g_score(start)=0;

f_score(start)=g_score(start)+heuristic_est(start,goal);

while ~isempty(OpenSet)
    [~,currInOpenSet]=min(f_score(OpenSet));
    currNode = OpenSet(currInOpenSet);
    if currNode == goal
        
        
        Route = goal;
        while (Route(1)~=start)
            Route=[CameFrom(Route(1)),Route];
        end
        
        return
    end
    OpenSet = OpenSet(OpenSet~=currNode);       %Remove current node
    ClosedSet = [ClosedSet, currNode];          %Add to closed set
    for neigh = RoadGraph{currNode}
        if ~sum(ClosedSet == neigh)             %If neighbor is not in closed set
            tentative_g = g_score(currNode) + RoadCost(currNode,neigh);
            if (~sum(neigh == OpenSet))         %If the neighbor is not already in OpenSet, add it
                OpenSet = [OpenSet, neigh];     
            elseif tentative_g>=g_score(neigh)
                continue
            end
            CameFrom(neigh) = currNode;
            g_score(neigh) = tentative_g;
            f_score(neigh) = g_score(neigh)+heuristic_est(neigh,goal);
            
        end
    end
end

