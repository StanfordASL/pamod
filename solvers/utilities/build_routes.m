function [RouteTime,RouteCharge,Routes] = build_routes(RoadGraph,TravelTimes,ChargeToTraverse)

N=length(RoadGraph);

RouteTime=zeros(N);
RouteCharge=zeros(N);
Routes=cell(N,N);

for i=1:N
    parfor j=1:N
        Routes{i,j} = FreeRouteAstar(i,j,RoadGraph,TravelTimes)
        for k=1:length(Routes{i,j})-1
            RouteTime(i,j)=RouteTime(i,j)+TravelTimes(Routes{i,j}(k),Routes{i,j}(k+1));
            RouteCharge(i,j)=RouteCharge(i,j)+ChargeToTraverse(Routes{i,j}(k),Routes{i,j}(k+1));
        end
    end
end

