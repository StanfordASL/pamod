function [expectedFlows]=estimateArrivalDistribution(t,settings,RoadNetwork,Trips,ArrivalsTracker,mode)

% estimate pax flows.
% Mode 0: estimate from previous window of size Thor*forecastPeriods
% Mode 1: use exact routes

Thor=settings.Thor;
forecastPeriods=settings.forecastPeriods;

NodesLocation=RoadNetwork.NodesLocation;
StationLocation=RoadNetwork.StationLocation;
numStations=length(RoadNetwork.StationNodeID);

arrivalTimes = Trips.arrivalTimes;
MData = Trips.MData;



if mode == 0 %estimate
    expectedFlows=zeros(numStations,numStations);
    for tfc=1:min(t,Thor*forecastPeriods)
        expectedFlows=expectedFlows+ArrivalsTracker{tfc};
    end
    expectedFlows=expectedFlows/tfc;
    
    expectedFlows=expectedFlows*Thor;
elseif mode ==1 %Exact
    
    expectedFlows=zeros(numStations,numStations);
    myCustomers = MData(arrivalTimes>=t & arrivalTimes<t+Thor,:);
    for ccTmp = 1:length(myCustomers)
        tmpCust = [myCustomers(ccTmp,6:7); myCustomers(ccTmp,8:9)]*1000; %km to m
        % find the nearest nodes
        tmpNodes = dsearchn(NodesLocation, tmpCust);
        if tmpNodes(1) ~= tmpNodes(2) % Check that start/end nodes are not the same
            % find the stations
            tmpStations = dsearchn(StationLocation, tmpCust);
            % make customer structure
            %customer(cc) = struct('opos',NodesLocation(tmpNodes(1),:), 'dpos', NodesLocation(tmpNodes(2),:),...
            %    'onode', tmpNodes(1), 'dnode', tmpNodes(2), 'ostation',tmpStations(1), 'dstation', tmpStations(2), ...
            %    'waitTime',0,'serviceTime',0,'pickedup',0,'delivered',0);
            % add this customer to the station
            %station(customer(cc).ostation).custId = [station(customer(cc).ostation).custId  cc];
            %station(customer(cc).ostation).custUnassigned = [station(customer(cc).ostation).custUnassigned  cc];
            % add to the arrival tracker
            expectedFlows(tmpStations(1), tmpStations(2)) = expectedFlows(tmpStations(1), tmpStations(2)) + 1;
        end
        
    end
end
