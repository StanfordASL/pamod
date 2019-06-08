function [rebPaths, paxPaths,vown_withpax] = iCRRP_MPC(vown, outstandingPax, expectedFlows,RoadNetwork,Thor)

%   vown, where vown(i) is the number of vehicles ownd by station i
%   outstandingPax(i,j) the number of passengers going from stations i to station j
%   expectedFlows(i,j) the expected number of passengeers from station i to station j within the next rebalancing period
%   please return rebPaths cell of cells where rebPaths{i} returns a cell array of paths for station i
%   please return paxPaths 2D cell of cells where paxPaths{i,j} returns a cell array of paths for origin station i, to destination j

RoadGraph=RoadNetwork.RoadGraph;
RoadCap=RoadNetwork.RoadCap;
LinkTime=RoadNetwork.LinkTime;
LinkLength = RoadNetwork.LinkLength;
LinkFreeFlow = RoadNetwork.LinkFreeFlow;
StationNodeID = RoadNetwork.StationNodeID;

%So, RoadCap is defined in vehicles. We want vehicles/unit time. To that
%end, we compute vehicles/m by dividing by roadLength. We then multiply by
%roadSpeed to get vehicles/s. Finally, we multiply by the duration of the
%time window of interest.
for i=1:size(RoadCap,1)
    for j=1:size(RoadCap,2)
        if RoadCap(i,j)>0
            RoadCap(i,j)=RoadCap(i,j)./LinkLength(i,j).*LinkFreeFlow(i,j)*Thor;
        end
    end
end
RoadCap=round(RoadCap);

% Compute the flows you want
nStations=length(StationNodeID);

Sources={};
Sinks={};
FlowsIn={};
FlowsOut={};
FlowCounter=1;
if (size(outstandingPax,1)~= size(expectedFlows,1)) || (size(outstandingPax,2)~= size(expectedFlows,2))
    disp('ERROR: outstandingPax and expectedFlows should have the same size')
    return
end
% Passenger flows

for i=1:size(expectedFlows,1)
    if (sum(outstandingPax(i,:)) + sum(expectedFlows(i,:))) >0
        Sources{FlowCounter}=[StationNodeID(i)];
        FlowsIn{FlowCounter}= (sum(outstandingPax(i,:)) + sum(expectedFlows(i,:)));
        Sinks{FlowCounter}=[];
        FlowsOut{FlowCounter}=[];
        for j=1:size(outstandingPax,2)
            if outstandingPax(i,j)>0 || expectedFlows(i,j)>0
                Sinks{FlowCounter}=[Sinks{FlowCounter}, StationNodeID(j)];
                FlowsOut{FlowCounter}= [FlowsOut{FlowCounter}, outstandingPax(i,j)+expectedFlows(i,j)];
            end
        end
        FlowCounter=FlowCounter+1;
    end
end
% Rebalancing flow

% Here, we rebalance the pax flows AND the excess vehicles. That is, we
% define vown_withpax= vehicles owned by the station - pax leaving the station +
% pax going to the station.

vown_withpax = vown - sum(outstandingPax,2) - sum(expectedFlows,2) + (sum(outstandingPax,1)') + (sum(expectedFlows,1)');
%disp('IGNORING PAX WHEN REBALANCING')
%vown_withpax=vown;
vdesired = floor(sum(vown_withpax)/length(vown))*ones(size(vown));
vdiff=sum(vown)-sum(vdesired);
while vdiff>0
    ri=randi(length(vdesired));
    vdesired(ri)=vdesired(ri)+1;
    vdiff=vdiff-1;
end
vexcess= vown_withpax-vdesired;
Sources{FlowCounter}=[];
FlowsIn{FlowCounter}=[];
Sinks{FlowCounter}=[];
FlowsOut{FlowCounter}=[];
for i=1:length(vexcess)
    if vexcess(i)>0 
        Sources{FlowCounter}=[Sources{FlowCounter}, StationNodeID(i)];
        FlowsIn{FlowCounter}=[FlowsIn{FlowCounter}, vexcess(i)];
    elseif vexcess(i)<0
        Sinks{FlowCounter}=[Sinks{FlowCounter}, StationNodeID(i)];
        FlowsOut{FlowCounter}=[FlowsOut{FlowCounter}, -vexcess(i)];
    end
end



% Call TIUnbalancedFlow and get the output

CommodityWeights=ones(length(Sources),1);
CommodityWeights(end)=0; %No cost for rebalancing

milpflag=0;
congrelaxflag=1;
sourcerelaxflag=0;
debugflag=1;


cplex_out = TIUnbalancedFlow(RoadGraph,RoadCap,LinkTime,Sources,Sinks,FlowsIn,FlowsOut,CommodityWeights,milpflag,congrelaxflag,sourcerelaxflag,debugflag);


% Split up the output. Call TIRebPathDecomposition_f on each pax path separately to get the
% output

TIUnbalancedFlowFinder
allpaths={};
pathsnodeindex={};
for k=1:M-1 %no reb flow
    %k
    % extract the relevant chunk of the LP output
    pax_out = cplex_out(Flowkij(k,1,RoadGraph{1}(1)):Flowkij(k,N,RoadGraph{N}(end)));
    paxSources=Sources{k};
    paxSinks=Sinks{k};
    %disp('FlowsIn size')
    %size(FlowsIn{k})
    %size(cplex_out(SoRelaxkl(k,1):SoRelaxkl(k,length(Sources{k}))))
    %disp('FlowsOut size')
    %size(FlowsOut{k})
    %size(cplex_out(SiRelaxkl(k,1):SiRelaxkl(k,length(Sinks{k}))))
    
    paxFlowsIn=FlowsIn{k}-cplex_out(SoRelaxkl(k,1):SoRelaxkl(k,length(Sources{k})))';
    paxFlowsOut=FlowsOut{k}-cplex_out(SiRelaxkl(k,1):SiRelaxkl(k,length(Sources{k})))';
    [allpaths{k},pathnodeindex{k}] = TIRebPathDecomposition_f(pax_out,RoadGraph,N,M,paxSources,paxSinks,paxFlowsIn,paxFlowsOut);
end
paxPathsToSample=cell(nStations,nStations);
paxWeightsToSample=cell(nStations,nStations);
for i=1:nStations
    for j=1:nStations
        paxPathsToSample{i,j}={};
    end
end

%Unpackage passenger paths
for k=1:length(allpaths)
    for l=1:length(allpaths{k}) %should always be 1
        for p=1:length(allpaths{k}{l})
            startnode=allpaths{k}{l}{p}(1,1);
            endnode = allpaths{k}{l}{p}(end,1);
            startstation = find(StationNodeID == startnode,1);
            endstation = find(StationNodeID == endnode,1);
            pathweight = allpaths{k}{l}{p}(end,2);
            paxPathsToSample{startstation,endstation}=[paxPathsToSample{startstation,endstation}, allpaths{k}{l}{p}(:,1)];
            paxWeightsToSample{startstation,endstation}=[paxWeightsToSample{startstation,endstation},pathweight];
        end
    end
end

% For each customer, sample a path wp equal to path weight.
paxPaths=cell(nStations,nStations);
for i=1:nStations
    for j=1:nStations
        paxPaths{i,j}={};
    end
end
for i=1:nStations
    for j=1:nStations
        if ~isempty(paxPathsToSample{i,j})
            numPaths=sum(paxWeightsToSample{i,j});
            probs = paxWeightsToSample{i,j}/numPaths;
            cumprobs = cumsum(probs);
            if abs(numPaths-round(numPaths))>=1e-8
                fprintf('Assert will fail in a moment: for flow %d-%d, I have an overall path weight of %f\n',i,j,numPaths)
                fprintf('Outstanding pax on %d-%d: %f, expected flow: %f\n',i,j,outstandingPax(i,j),expectedFlows(i,j))
            end
            assert(abs(numPaths-round(numPaths))<1e-8,'non-integer number of paths, exiting')
            for pa=1:round(numPaths)
                rprob=rand();
                %fprintf('Path from %d to %d: rand. sample %f, vector of cum probs:\n',i, j,rprob)
                %disp(cumprobs)
                if length(probs)==1
                    %fprintf('One possible path:\n')
                    %disp(paxPathsToSample{i,j})
                    sampledpath = paxPathsToSample{i,j}{1};
                else
                    for pp=1:length(probs)
                        %fprintf('Possible paths:\n')
                        %paxPathsToSample{i,j}
                        if  rprob < cumprobs(pp)
                            sampledpath = paxPathsToSample{i,j}{pp};
                            break
                        end
                    end
                end
                paxPaths{i,j}=[paxPaths{i,j}, sampledpath ];
            end
        end
    end
end

% Compute residual capacity
RebRoadCap=RoadCap;
for i=1:nStations
    for j=1:nStations
        if ~isempty(paxPaths{i,j})
            if iscell(paxPaths{i,j})
                for pathid=1:length(paxPaths{i,j})
                    tpath = paxPaths{i,j}{pathid};
                    for no=1:length(tpath)-1
                        RebRoadCap(tpath(no),tpath(no+1))=RebRoadCap(tpath(no),tpath(no+1))-1;
                    end
                end
            else %if only one path is there, it may not be in cell format
                tpath = paxPaths{i,j};
                for no=1:length(tpath)-1
                        RebRoadCap(tpath(no),tpath(no+1))=RebRoadCap(tpath(no),tpath(no+1))-1;
                end
            end
        end
    end
end
RebRoadCap=max(RebRoadCap,zeros(size(RebRoadCap)));

%call TIMultiCommodityFlow with modified slack variables to get reb flow.
milpflag=0;
congrelaxflag=1;
sourcerelaxflag=0;
debugflag=1;

if sum(FlowsIn{end}) ~= sum(FlowsOut{end})
    FlowsIn{end}
    FlowsOut{end}
    fprintf('WARNING: Reb Sources do not match Reb Sinks, diff = %d, FlowsIn = %d, FlowsOut = %d \n', sum(FlowsIn{end}) - sum(FlowsOut{end}), FlowsIn{end}, FlowsOut{end})
end

cplex_out_reb = TIUnbalancedFlow(RoadGraph,RebRoadCap,LinkTime,{Sources{end}},{Sinks{end}},{FlowsIn{end}},{FlowsOut{end}},[1],milpflag,congrelaxflag,sourcerelaxflag,debugflag);

% Decompose reb flow

% extract the relevant chunk of the LP output
rebSources=Sources{end};
rebSinks=Sinks{end};

rebFlowsIn=FlowsIn{end}-cplex_out_reb(2*E+1:2*E+length(Sources{end}))';
rebFlowsOut=FlowsOut{end}-cplex_out_reb(2*E+length(Sources{end})+1:end)';

[rebPathsBulk,rebpathnodeindex] = TIRebPathDecomposition_f(cplex_out_reb,RoadGraph,N,1,rebSources,rebSinks,rebFlowsIn,rebFlowsOut);

% Clean up paths so we have UNIT paths in rebpaths{i}
rebPaths=cell(nStations);
for i=1:nStations
    rebPaths{i}={};
end
for i=1:length(rebPathsBulk)
    
    if iscell(rebPathsBulk{i})
        for p=1:length(rebPathsBulk{i})
            tpath = rebPathsBulk{i}{p};
            startstation = find(StationNodeID==tpath(1,1),1);
            %if (StationNodeID(i) ~= tpath(1,1))
            %    fprintf('Will fail an assert. Considering rebpathscell %d, which should correspond to station %d and node %d\nHowever, path departs %d\n',i,i,StationNodeID(i),tpath(1,1) )
            %end
            %assert(StationNodeID(i) == tpath(1,1),'ERROR: this path does not start from where it should')
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

%% Sanity check on output
for i=1:size(expectedFlows,1)
    for j=1:size(expectedFlows,2)
        if length(paxPaths{i,j})~=expectedFlows(i,j)+outstandingPax(i,j)
           fprintf('WARNING: number of paths from %d to %d is different than requested (desired: %d, actual %d)\n',i,j,expectedFlows(i,j)+outstandingPax(i,j),length(paxPaths{i,j}))
        end
    end
end