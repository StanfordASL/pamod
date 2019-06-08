function [assignedCarID,carOnRoadIndex] = AssignCustomerToCar(ThisCustomer,ThisStation,ThisStationLocation,car,RoadNetwork,RebOptions)

%fprintf('\n\n WARNING: AssignCustomerToCar is charge-unaware. Fix that!\n')

distToStation = norm(ThisCustomer.opos - ThisStationLocation);

custRequiredCharge = RoadNetwork.RouteCharge(RoadNetwork.StationNodeID(ThisCustomer.ostation),RoadNetwork.StationNodeID(ThisCustomer.dstation));

assignedCarID=-1;
% get distance to nearest car on road
% This method exclusively checks for vehicles on the station
% NOTE: what if there are no vehicles (road or idle) in the station?
%		This is specially worrisome when the number of stations is large
%		Since it is more likely that they will be empty
%		Also, the match is first pass (i.e. no matching algorithm)
shortestDistance = inf;
if ~isempty(ThisStation.carIdle)
    %TODO assign customer to vehicle at the 'right' charge level (the optimizer tells us which). But what if an unexpected customer shows up? Do you give them the lowest charge level that will work?
    for cc=ThisStation.carIdle
        carCharge=ceil((car(cc).charge-RebOptions.chargeBias)/RebOptions.chargeProportion); % compute discretized charge level (use same data given to controller)
        if carCharge>custRequiredCharge+RebOptions.chargeBuffer
               assignedCarID = cc;
               shortestDistance = distToStation;
               break
        end
    end
end
carOnRoadIndex = 0;
if ~isempty(ThisStation.carOnRoad)
    for j = 1:length(ThisStation.carOnRoad)
        distToCar = norm(ThisCustomer.opos - car(ThisStation.carOnRoad(j)).pos);
        chargeToPaxPickup = RoadNetwork.RouteCharge(car(ThisStation.carOnRoad(j)).path(1),ThisCustomer.onode);
        carCharge=ceil((car(ThisStation.carOnRoad(j)).charge-RebOptions.chargeBias)/RebOptions.chargeProportion); % compute discretized charge level (use same data given to controller)
        if carCharge<=chargeToPaxPickup+custRequiredCharge
            continue
        end
        if distToCar < shortestDistance
            shortestDistance = distToCar;
            assignedCarID = ThisStation.carOnRoad(j);
            carOnRoadIndex = j;
        end
    end
end


assert(shortestDistance ~=inf | assignedCarID==-1,'ERROR: inf shortest distance')