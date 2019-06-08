function [rebPaths, paxPaths,logs] = CongestionAwareRebalancer(station,car,customer,LinkNumVehicles,LinkTime,RebOptions,RoadNetwork,PowerNetwork)

addpath(genpath('utilities'));

numStations = length(station);
v=length(car);
Thor=RebOptions.Thor;

LoadStateDefinitions;

vown = zeros(numStations,1);
outstandingPax = zeros(numStations, numStations);
custUnassigned = zeros(numStations,1);

for i = 1:numStations
    vown(i) = length(station(i).carIdle) + length(station(i).carOnRoad);
    if ~isempty(station(i).custUnassigned)
        for cindx = 1:length(station(i).custUnassigned)
            cust = customer(station(i).custUnassigned(cindx));
            outstandingPax(cust.ostation,cust.dstation) = outstandingPax(cust.ostation,cust.dstation) + 1;
        end
        custUnassigned(i)=length(station(i).custUnassigned);
    end
end
% We also count pax that have been assigned but not picked up
% as outstanding. We haven't assigned them a route, after all
for i = 1:v
    % if the car is in the process of pickup or dropoff
    if car(i).state == DRIVING_TO_DEST % || car(i).state == 1
        vown(car(i).dstation) = vown(car(i).dstation) + 1;
    end
    if car(i).state == DRIVING_TO_PICKUP
        outstandingPax(customer(car(i).passId).ostation, customer(car(i).passId).dstation)=outstandingPax(customer(car(i).passId).ostation, customer(car(i).passId).dstation)+1;
    end
end

%%%%%%%%%%% TODO: Federico, this is where we call your funciton.
%     I'm passing you vown, where vown(i) is the number of vehicles ownd by station i
%     outstandingPax(i,j) the number of passengers going from stations i to station j
%     expectedFlows(i,j) the expected number of passengeers from station i to station j within the next rebalancing period
%     please return rebPaths cell of cells where rebPaths{i} returns a cell array of paths for station i
%     please return paxPaths 2D cell of cells where paxPaths{i,j} returns a cell array of paths for origin station i, to destination j


% Mode 0: use new method for both pax and reb
% Mode 1: use CReP+A*
% Mode 2: use sampling for pax, CReP RSS-style for reb.
% if CRePFLAG~=1
%     % Mode 0: estimate flow from previous window
%     % Mode 1: exact knowledge of routes
%     expectedFlows=estimateArrivalDistribution(t,settings,RoadNetwork,Trips,ArrivalsTracker,1);
%     
%     [rebPaths, paxPaths,vown_withpax] = iCRRP_MPC(vown, outstandingPax, round(expectedFlows),RoadNetwork,Thor);
%     
%     vownlog_withpax{t}=vown_withpax;
%     %%% ACCOUNTING
%     numrebpaths=0;
%     for i = 1:length(rebPaths)
%         issuedRebPaths(i,t) = length(rebPaths{i});
%         numrebpaths=numrebpaths+length(rebPaths{i});
%     end
%     numComputedRebalancingRoutes(t)=numrebpaths;
% else %RSS
paxPaths = cell(numStations,numStations);
%end
%if CRePFLAG>0
RoadNetwork.LinkTime=LinkTime;
RoadNetwork.LinkSpeed=RoadNetwork.LinkLength/LinkTime;
RoadNetwork.LinkSpeed=sparse(~isnan(RoadNetwork.LinkSpeed));
[rebPaths]=iCReP_MPC(vown, custUnassigned, LinkNumVehicles ,RoadNetwork,Thor,0);
%end
%%% END:ACCOUNTING

logs.vownlog=vown;
logs.outstandingPaxlog=outstandingPax;
logs.custUnassigned=custUnassigned;
