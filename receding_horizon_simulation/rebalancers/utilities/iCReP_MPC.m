function [rebPaths]=iCReP_MPC(vown, custUnassigned, LinkNumVehicles ,RoadNetwork,Thor,NAIVEFLAG)

% Reimplementation of the RSS 2016 algorithm.


RoadGraph=RoadNetwork.RoadGraph;
RoadCap=RoadNetwork.RoadCap;
LinkTime=RoadNetwork.LinkTime;
LinkLength = RoadNetwork.LinkLength;
LinkSpeed = RoadNetwork.LinkSpeed;
StationNodeID = RoadNetwork.StationNodeID;

%So, RoadCap is defined in vehicles. We want vehicles/unit time. To that
%end, we compute vehicles/m by dividing by roadLength. We then multiply by
%roadSpeed to get vehicles/s. Finally, we multiply by the duration of the
%time window of interest.

LinkCapacityLeft = max(RoadCap - LinkNumVehicles, 0);

LinkCapacityLeft=LinkCapacityLeft./LinkLength.*LinkSpeed*Thor;


% Compute the flows you want
numStations=length(StationNodeID);

totalCustomers=0;

%% find excess vehicles of each station
for i = 1:numStations
    vexcess(i) = vown(i) - custUnassigned(i);
    % find total customers
    totalCustomers = totalCustomers + custUnassigned(i);
end

%% vehicles desired for each station
vdesired = floor((sum(vown) - totalCustomers)/numStations)*ones(numStations,1);
%
if NAIVEFLAG
    %    % car optimization
    %    cvx_begin quiet
    %        variable numij(numStations,numStations)
    %        minimize (sum(sum(Tij.*numij)));
    %        subject to
    %        vexcess + sum((numij' - numij)')' >= vdesired;
    %        % sum(numij')' <= rown;
    %        % trace(numij) == 0;
    %        numij >= 0;
    %    cvx_end
    %    % make sure numij is integer
    %    numij = round(numij);
    %    % add rebalancing vehicles to queues
    %    for i = 1:numStations
    %        for j = 1:numStations
    %            for k = 1:numij(i,j)
    %                rebalanceQueue{i} = [rebalanceQueue{i} j];
    %            end
    %        end
    %    end
else
    % calculate difference between the floor value and the actual
    % number
    vdesiredDifference = (sum(vown) - totalCustomers) - numStations*vdesired(1);
    [~, sortedIndex] = sort(vown);
    for i = 1:vdesiredDifference
        vdesired(sortedIndex(i)) = vdesired(sortedIndex(i)) + 1;
    end
    % form the sets S and T (sources and sinks) and the flows in/out
    sources = [];
    sinks = [];
    flowOut = [];
    flowIn = [];
    for i = 1:numStations
        tmpFlow = vexcess(i) - vdesired(i);
        if tmpFlow > 0
            for j = 1:tmpFlow
                % source
                sources = [sources, StationNodeID(i)];
                flowOut = [flowOut, 1];
            end
        elseif tmpFlow < 0
            for j = 1:-tmpFlow
                % sink
                sinks = [sinks, StationNodeID(i)];
                flowIn = [flowIn, 1];
            end
        end
    end
    %sum(flowOut)
    % find the link capacity left
    LinkCapacityLeft = max(RoadCap - LinkNumVehicles, 0);
    
    % call the single-commodity flow solver
    RebOutput = TIMulticommodityFlow(RoadGraph, LinkCapacityLeft, LinkTime , sources, sinks, flowIn, flowOut, 0, 0, 1);
    
    %This modifies the sources and sinks so the path planner knows that
    %some paths were modified
    S=length(sources);
    M=1;
    N=length(RoadGraph);
    SoRelaxkl=@(k,l) N*N*M + N*N + S*(k-1) + l;
    SiRelaxkl=@(k,l) N*N*M + N*N + M*S + S*(k-1) + l;
    relFlowIn = flowIn -RebOutput(SoRelaxkl(1,1):SoRelaxkl(1,S))';
    relFlowOut= flowOut-RebOutput(SiRelaxkl(1,1):SiRelaxkl(1,S))';
    %numVehiclesNotRebalancing(t) = sum(RebOutput(SoRelaxkl(1,1):SoRelaxkl(1,S)));
    [rebPathsBulk,pathnodeindex,npaths] = TIRebPathDecomposition(RebOutput, length(RoadGraph), 1, sources, sinks, relFlowIn, relFlowOut);
    disp('how much didnt rebalance')
    sum(flowOut) - length(rebPathsBulk)
    
    
    %Rearrange reb paths
    rebPaths=cell(numStations);
    for i=1:numStations
        rebPaths{i}={};
    end
    for i=1:length(rebPathsBulk)
        if iscell(rebPathsBulk{i})
            for p=1:length(rebPathsBulk{i})
                tpath = rebPathsBulk{i}{p};
                startstation = find(StationNodeID==tpath(1,1),1);
                if size(tpath,1)>1 %dirty data->sometimes we have one-step paths, which make zero sense
                    pathintensity = tpath(end,2);% number of unit paths
                    for cp=1:pathintensity
                        rebPaths{startstation}=[rebPaths{startstation},tpath(1:end,1)];
                    end
                end
            end
        else
            disp('Does this ever happen??')
            tpath = rebPathsBulk{i};
            startstation = find(StationNodeID==tpath(1,1),1);
            if size(tpath,1)>1 %dirty data->sometimes we have one-step paths, which make zero sense
                pathintensity = tpath(end,2);% number of unit paths
                for cp=1:pathintensity
                    rebPaths{startstation}=[rebPaths{startstation},tpath(1:end,1)];
                end
            end
        end
    end
end
