clear all; close all; clc;

addpath('utilities')
%% Horizon
Thor=10;

%% Build a road network
C=5;

%Lattice
number_rows=2;
number_cols=2;

[N,RoadGraph,NodesLocations] = build_road_lattice(number_rows,number_cols);

RoadCap=zeros(N);
for i=1:length(RoadGraph)
    for j=RoadGraph{i}
        if j~=i
            RoadCap(i,j)=10;
        else
            RoadCap(i,j)=Inf;
        end
    end
end
TVRoadCap=zeros(Thor,N,N);
for t=1:Thor
    TVRoadCap(t,:,:)=RoadCap;
end

TravelDistance=zeros(N);
for i=1:length(RoadGraph)
    for j=RoadGraph{i}
        if j~=i
            TravelDistance(i,j)=1;
        else
            TravelDistance(i,j)=0;
        end
    end
end

ChargeToTraverse=TravelDistance;

%Travel time is nonzero even on self loops
TravelTimes=zeros(N);
for i=1:length(RoadGraph)
    for j=RoadGraph{i}
        TravelTimes(i,j)=1;
    end
end



ChargersList = [4];
ChargerSpeed = [1];
ChargerTime  = [1];
ChargerCap   = [99];

ChargeUnitToPowerUnit = 1;
MinEndCharge           = 2;
ValueOfTime            = 1;
VehicleCostPerKm       = 1;

[RouteTime,RouteCharge,Routes] = build_routes(RoadGraph,TravelTimes,ChargeToTraverse);

%% RebWeight
RebWeight=.00;

%% Demand

% Sources=[1;1;2];
% Sinks=[3;4;4];
% StartTimes=[1;1;2];
% FlowsIn=[1;1;1];
Sources=[1, 1];
Sinks=[4, 4];
StartTimes=[1, 1];
FlowsIn=[1,1];

% Modify source data. Note that for the TV version we'd want to keep unique
% (source, start time) pairs

CompactSinks=      unique(Sinks);
CompactSources=    cell(size(CompactSinks));
CompactStartTimes= cell(size(CompactSinks));
CompactFlows=      cell(size(CompactSinks));



for i=1:length(CompactSinks);
    CompactSources{i}=Sources(Sinks==CompactSinks(i));
    CompactFlows{i}=FlowsIn(Sinks==CompactSinks(i));
    CompactStartTimes{i}=StartTimes(Sinks==CompactSinks(i));
end

M=length(CompactSinks);

Sources=CompactSources;
Sinks=CompactSinks';
FlowsIn=CompactFlows';
StartTimes=CompactStartTimes';

StarterSources=1;
StarterSinks=2;
StarterFlows=0;

CompactStarterSinks=      unique(StarterSinks);
CompactStarterSources=    cell(size(CompactStarterSinks));
CompactStarterFlows=      cell(size(CompactStarterSinks));

for i=1:length(CompactStarterSinks);
    CompactStarterSources{i}=StarterSources(StarterSinks==CompactStarterSinks(i));
    CompactStarterFlows{i}=StarterFlows(StarterSinks==CompactStarterSinks(i));
end

StarterSources=CompactStarterSources;
StarterSinks=CompactStarterSinks';
StarterFlows=CompactStarterFlows';
%% Empty initial position of vehicles

EmptyVehicleInitialPosRT=zeros(Thor,N,C);
%EmptyVehicleInitialPosRT(1,1,C)=2;
EmptyVehicleInitialPosRT(1,1,3)=2;
EmptyVehicleInitialPosRT(1,2,4)=0;

EmptyVehicleInitialPos=zeros(N,C);
% EmptyVehicleInitialPos(1,C)=0;
% EmptyVehicleInitialPos(2,C)=1;
% EmptyVehicleInitialPos(3,C)=1;

FullVehicleInitialPos=zeros(M,N,C);

MinNumVehiclesAti=zeros(N,1);

%MinNumVehiclesAti(1) = 2;
%% Power graph
PowerGraph={[2, 4],[],[4, 2],[]};
NP=length(PowerGraph);

PowerLineCap=cell(size(PowerGraph));
PowerLineReactance=cell(size(PowerGraph));
for i=1:length(PowerGraph)
    PowerLineCap{i}=zeros(size(PowerGraph{i}));
    PowerLineReactance{i}=zeros(size(PowerGraph{i}));
    for j=1:length(PowerGraph{i})
        PowerLineCap{i}(j)=10;
        PowerLineReactance{i}(j)=1;
    end
end
%PowerLineCap=999*ones(2);

PowerLineReactance=PowerLineCap;

PowerGensList=[2,4];
NumGenerators=length(PowerGensList);
PowerGensMax=ones(Thor,NumGenerators)*10;
PowerGensMin=ones(Thor,NumGenerators)*0;
PowerCosts=ones(Thor,NumGenerators);
PowerCosts(:,1)=.49;
%PowerCosts=PowerCosts*1e6;
PowerExtLoads=zeros(NP,Thor);
PowerExtLoads(1,4)=20;
%PowerExtLoads(2,5)=21;
%PowerExtLoads(2)=1;
RoadToPowerMap=[1, 3];
v2g_efficiency=.9;
PowerRampUp=ones(Thor,NumGenerators)*10;
PowerRampDown=ones(Thor,NumGenerators)*10;

%% Flags

milpflag=0;
congrelaxflag=0;
sourcerelaxflag=1;
cachedAeqflag=0;
solverflag='CPLEX';
minendchargerelaxflag=0;
endreblocrelaxflag=1;
powernetworkloadrelaxflag=0;

%%
RoadNetwork.C=C;
RoadNetwork.RoadGraph=RoadGraph;
RoadNetwork.TVRoadCap=TVRoadCap;
RoadNetwork.TravelTimes=TravelTimes;
RoadNetwork.TravelDistance=TravelDistance;
RoadNetwork.ChargeToTraverse=ChargeToTraverse;
RoadNetwork.ChargersList=ChargersList;
RoadNetwork.ChargerSpeed=ChargerSpeed;
RoadNetwork.ChargerTime=ChargerTime;
RoadNetwork.ChargerCap=ChargerCap;
RoadNetwork.ChargeUnitToPowerUnit=ChargeUnitToPowerUnit;
RoadNetwork.MinEndCharge=MinEndCharge;
RoadNetwork.ValueOfTime=ValueOfTime;
RoadNetwork.VehicleCostPerKm=VehicleCostPerKm;
RoadNetwork.RouteTime=RouteTime;
RoadNetwork.RouteCharge=RouteCharge;

RoadNetwork.RoadCap=RoadCap;

InitialConditions.EmptyVehicleInitialPosRT=EmptyVehicleInitialPosRT;
InitialConditions.MinNumVehiclesAti=MinNumVehiclesAti;

InitialConditions.EmptyVehicleInitialPos=EmptyVehicleInitialPos;
InitialConditions.FullVehicleInitialPos=FullVehicleInitialPos; 

Passengers.Sources=Sources;
Passengers.Sinks=Sinks;
Passengers.Flows=FlowsIn;
Passengers.StartTimes=StartTimes;
Passengers.StarterSources=StarterSources;
Passengers.StarterSinks=StarterSinks;
Passengers.StarterFlows=StarterFlows;

Passengers.StarterSinks=[1];
Passengers.StarterSources={[1]};
Passengers.StarterFlows={[0]};

PowerNetwork.PowerGraphM=PowerGraph;
PowerNetwork.PowerLineCapM=PowerLineCap;
PowerNetwork.PowerLineReactanceM=PowerLineReactance;
PowerNetwork.PowerGensList=PowerGensList;
PowerNetwork.PowerGensMax=PowerGensMax;
PowerNetwork.PowerGensMin=PowerGensMin;
PowerNetwork.PowerCosts=PowerCosts;
PowerNetwork.PowerExtLoads=PowerExtLoads;
PowerNetwork.RoadToPowerMap=RoadToPowerMap;
PowerNetwork.v2g_efficiency=v2g_efficiency;
PowerNetwork.PowerRampUp=PowerRampUp;
PowerNetwork.PowerRampDown=PowerRampDown;

Flags.milpflag=milpflag;
Flags.congrelaxflag=congrelaxflag;
Flags.sourcerelaxflag=sourcerelaxflag;
Flags.cachedAeqflag=cachedAeqflag;
Flags.solverflag=solverflag;
Flags.minendchargerelaxflag=minendchargerelaxflag;
Flags.endreblocrelaxflag=endreblocrelaxflag;
Flags.powernetworkloadrelaxflag=powernetworkloadrelaxflag;

%%
%[cplex_out,fval,exitflag,output,lambdas,dual_prices_ix,dual_charger_prices_ix]=TVPowerBalancedFlow_withpower_sinkbundle(Thor,RoadNetwork,PowerNetwork,InitialConditions,RebWeight,Passengers,Flags);
%%
[cplex_out,fval,exitflag,output,lambdas,dual_prices_ix,dual_charger_prices_ix]=TVPowerBalancedFlow_realtime(Thor,RoadNetwork,PowerNetwork,InitialConditions,RebWeight,Passengers,Flags);

%% Test sampling
[RebPaths,PaxChargeLevels]=TVPowerBalancedFlowSampler_realtime(cplex_out,Thor,RoadNetwork,PowerNetwork,InitialConditions,RebWeight,Passengers,Flags);

%% Output
DIAGNOSTIC_FLAG=0;
TVPowerBalancedFlowFinder_realtime;

for t=1:Thor
    fprintf('At time t=%d..\n',t)
    % Waiting passengers
    for k=1:MS
        for c=1:C 
            for ssi=1:length(StarterSources{k})
                if abs(cplex_out(FindStarterSourceChargetcks(t,c,k,ssi)))>1e-5
                    fprintf('%f waiting passenger(s) depart(s) from %d to %d at charge level %d\n',cplex_out(FindStarterSourceChargetcks(t,c,k,ssi)),StarterSources{k}(ssi),StarterSinks(k),c)
                end
            end
        end
        for c=1:C
            if cplex_out(FindStarterSinkChargetck(t,c,k))>=1e-5
                fprintf('%f waiting passenger(s) arrive(s) at %d at charge level %d\n',cplex_out(FindStarterSinkChargetck(t,c,k)),StarterSinks(k),c)
            end
        end
    end
    % Passengers
    for k=1:M
        for ssi=1:length(Sources{k})
            for c=1:C
                if abs(cplex_out(FindPaxSourceChargecks(c,k,ssi)))>=1e-5 && StartTimes{k}(ssi)==t
                    fprintf('%f passenger(s) departs from %d to %d at charge level %d\n',cplex_out(FindPaxSourceChargecks(c,k,ssi)),Sources{k}(ssi),Sinks(k),c)
                end
            end
        end
        for c=1:C
            if cplex_out(FindPaxSinkChargetck(t,c,k))>=1e-5
                fprintf('%f passenger(s) arrives at %d at charge level %d\n',cplex_out(FindPaxSinkChargetck(t,c,k)),Sinks(k),c)
            end
        end
    end
    % Reb. vehicles: travel
    for i=1:N
        for j=RoadGraph{i}
            for c=1:C
                if (abs(cplex_out(FindRoadLinkRtcij(t,c,i,j)))>=1e-5)
                    if j~=i
                        fprintf('%f reb. vehicle(s) drive from %d to %d at charge level %d\n',cplex_out(FindRoadLinkRtcij(t,c,i,j)),i,j,c)
                    else
                        %fprintf('%f reb. vehicle(s) stay at %d at charge level %d\n',cplex_out(FindRoadLinkRtcij(t,c,i,j)),i,c)
                    end
                end
            end
        end
    end
    % Reb. vehicles: charge and discharge
    for l=1:NumChargers
        for c=1:C
           if (abs(cplex_out(FindChargeLinkRtcl(t,c,l)))>=1e-5)
               fprintf('%f reb. vehicle(s) charge at %d at charge level %d\n',cplex_out(FindChargeLinkRtcl(t,c,l)),ChargersList(l),c);
           end
        end
    end
    for l=1:NumChargers
        for c=1:C
           if (abs(cplex_out(FindDischargeLinkRtcl(t,c,l)))>=1e-5)
               fprintf('%f reb. vehicle(s) discharges at %d at charge level %d\n',cplex_out(FindDischargeLinkRtcl(t,c,l)),ChargersList(l),c);
           end
        end
    end
    % Generators
    for i=1:NumGenerators
       if (abs(cplex_out(FindGeneratorti(t,i)))>=1e-5)
           fprintf('Generator %d produces %f energy\n',i,cplex_out(FindGeneratorti(t,i)))
       end
    end
end
fprintf('End locations:\n')
for i=1:N
    for c=1:C
        if abs(cplex_out(FindEndRebLocationci(c,i)))>=1e-5
            fprintf('%f vehicle(s) at %d with charge %d\n',cplex_out(FindEndRebLocationci(c,i)),i,c)
        end
    end
end
if sourcerelaxflag
    fprintf('Dropped pax:\n')
    for k=1:M
        for ssi=1:length(Sources{k})
            if abs(cplex_out(FindSourceRelaxks(k,ssi)))>=1e-5
                fprintf('Dropped %d pax from %d to %d at time %d\n',cplex_out(FindSourceRelaxks(k,ssi)),Sources{k}(ssi),Sinks(k),StartTimes{k}(ssi))
            end
        end
    end
    for k=1:MS
        for ssi=1:length(StarterSources{k})
            if abs(cplex_out(FindStarterSourceRelaxks(k,ssi)))>=1e-5
                fprintf('Dropped %d waiting pax from %d to %d\n',cplex_out(FindStarterSourceRelaxks(k,ssi)),StarterSources{k}(ssi),StarterSinks(k))
            end
        end
    end
end
if endreblocrelaxflag
    for i=1:N
        if abs(cplex_out(FindEndRebLocRelaxi(i)))>=1e-5
            fprintf('Relaxed end location constraint by %d at node %d\n',cplex_out(FindEndRebLocRelaxi(i)),i)
        end
    end
end
if powernetworkloadrelaxflag
    for t=1:Thor
        for i=1:NP
            if abs(cplex_out(FindPowerNetworkRelaxti(t,i)))>=1e-5
                fprintf('Relaxed power load by %d at time %d at node %d\n',cplex_out(FindPowerNetworkRelaxti(t,i)),t,i)
            end
        end
    end
end

% Tests:
% Move reb. vehicles and pick up pax at 1
% Require recharging before moving to pick up pax
% Multiple charging locations w. different prices
% Multiple charging locations w. constraints
% Background power loads
% Interconnected power network loads





%% Test 2
disp('Ready for test 2, press a key to continue')
pause
%% Horizon
Thor=10;

%% Build a road network
C=5;

%Lattice
number_rows=2;
number_cols=2;

[N,RoadGraph,NodesLocations] = build_road_lattice(number_rows,number_cols);

RoadCap=zeros(N);
for i=1:length(RoadGraph)
    for j=RoadGraph{i}
        if j~=i
            RoadCap(i,j)=10;
        else
            RoadCap(i,j)=Inf;
        end
    end
end
TVRoadCap=zeros(Thor,N,N);
for t=1:Thor
    TVRoadCap(t,:,:)=RoadCap;
end

TravelDistance=zeros(N);
for i=1:length(RoadGraph)
    for j=RoadGraph{i}
        if j~=i
            TravelDistance(i,j)=1;
        else
            TravelDistance(i,j)=0;
        end
    end
end

ChargeToTraverse=TravelDistance;

%Travel time is nonzero even on self loops
TravelTimes=zeros(N);
for i=1:length(RoadGraph)
    for j=RoadGraph{i}
        TravelTimes(i,j)=1;
    end
end



ChargersList = [4];
ChargerSpeed = [1];
ChargerTime  = [1];
ChargerCap   = [99];

ChargeUnitToPowerUnit = 1;
MinEndCharge           = 2;
ValueOfTime            = 1;
VehicleCostPerKm       = 1;

[RouteTime,RouteCharge,Routes] = build_routes(RoadGraph,TravelTimes,ChargeToTraverse);

%% RebWeight
RebWeight=.00;

%% Demand

% Sources=[1;1;2];
% Sinks=[3;4;4];
% StartTimes=[1;1;2];
% FlowsIn=[1;1;1];
Sources=[1, 2];
Sinks=[4, 4];
StartTimes=[1, 2];
FlowsIn=[1,1];

% Modify source data. Note that for the TV version we'd want to keep unique
% (source, start time) pairs

CompactSinks=      unique(Sinks);
CompactSources=    cell(size(CompactSinks));
CompactStartTimes= cell(size(CompactSinks));
CompactFlows=      cell(size(CompactSinks));



for i=1:length(CompactSinks);
    CompactSources{i}=Sources(Sinks==CompactSinks(i));
    CompactFlows{i}=FlowsIn(Sinks==CompactSinks(i));
    CompactStartTimes{i}=StartTimes(Sinks==CompactSinks(i));
end

M=length(CompactSinks);

Sources=CompactSources;
Sinks=CompactSinks';
FlowsIn=CompactFlows';
StartTimes=CompactStartTimes';

StarterSources=1;
StarterSinks=2;
StarterFlows=0;

CompactStarterSinks=      unique(StarterSinks);
CompactStarterSources=    cell(size(CompactStarterSinks));
CompactStarterFlows=      cell(size(CompactStarterSinks));

for i=1:length(CompactStarterSinks);
    CompactStarterSources{i}=StarterSources(StarterSinks==CompactStarterSinks(i));
    CompactStarterFlows{i}=StarterFlows(StarterSinks==CompactStarterSinks(i));
end

StarterSources=CompactStarterSources;
StarterSinks=CompactStarterSinks';
StarterFlows=CompactStarterFlows';
%% Empty initial position of vehicles

EmptyVehicleInitialPosRT=zeros(Thor,N,C);
%EmptyVehicleInitialPosRT(1,1,C)=2;
EmptyVehicleInitialPosRT(1,1,3)=0;
EmptyVehicleInitialPosRT(1,2,4)=2;

EmptyVehicleInitialPos=zeros(N,C);
% EmptyVehicleInitialPos(1,C)=0;
% EmptyVehicleInitialPos(2,C)=1;
% EmptyVehicleInitialPos(3,C)=1;

FullVehicleInitialPos=zeros(M,N,C);

MinNumVehiclesAti=zeros(N,1);

%MinNumVehiclesAti(1) = 2;
%% Power graph
PowerGraph={[2, 4],[],[4, 2],[]};
NP=length(PowerGraph);

PowerLineCap=cell(size(PowerGraph));
PowerLineReactance=cell(size(PowerGraph));
for i=1:length(PowerGraph)
    PowerLineCap{i}=zeros(size(PowerGraph{i}));
    PowerLineReactance{i}=zeros(size(PowerGraph{i}));
    for j=1:length(PowerGraph{i})
        PowerLineCap{i}(j)=10;
        PowerLineReactance{i}(j)=1;
    end
end
%PowerLineCap=999*ones(2);

PowerLineReactance=PowerLineCap;

PowerGensList=[2,4];
NumGenerators=length(PowerGensList);
PowerGensMax=ones(Thor,NumGenerators)*10;
PowerGensMin=ones(Thor,NumGenerators)*0;
PowerCosts=ones(Thor,NumGenerators);
PowerCosts(:,1)=.49;
%PowerCosts=PowerCosts*1e6;
PowerExtLoads=zeros(NP,Thor);
PowerExtLoads(1,4)=20;
%PowerExtLoads(2,5)=21;
%PowerExtLoads(2)=1;
RoadToPowerMap=[1, 3];
v2g_efficiency=.9;
PowerRampUp=ones(Thor,NumGenerators)*10;
PowerRampDown=ones(Thor,NumGenerators)*10;

%% Flags

milpflag=0;
congrelaxflag=0;
sourcerelaxflag=1;
cachedAeqflag=0;
solverflag='CPLEX';
minendchargerelaxflag=0;
endreblocrelaxflag=1;
powernetworkloadrelaxflag=0;
cyclicflag = 3;

%%
RoadNetwork.C=C;
RoadNetwork.RoadGraph=RoadGraph;
RoadNetwork.TVRoadCap=TVRoadCap;
RoadNetwork.TravelTimes=TravelTimes;
RoadNetwork.TravelDistance=TravelDistance;
RoadNetwork.ChargeToTraverse=ChargeToTraverse;
RoadNetwork.ChargersList=ChargersList;
RoadNetwork.ChargerSpeed=ChargerSpeed;
RoadNetwork.ChargerTime=ChargerTime;
RoadNetwork.ChargerCap=ChargerCap;
RoadNetwork.ChargeUnitToPowerUnit=ChargeUnitToPowerUnit;
RoadNetwork.MinEndCharge=MinEndCharge;
RoadNetwork.ValueOfTime=ValueOfTime;
RoadNetwork.VehicleCostPerKm=VehicleCostPerKm;
RoadNetwork.RouteTime=RouteTime;
RoadNetwork.RouteCharge=RouteCharge;

RoadNetwork.RoadCap=RoadCap;

InitialConditions.EmptyVehicleInitialPosRT=EmptyVehicleInitialPosRT;
InitialConditions.MinNumVehiclesAti=MinNumVehiclesAti;

InitialConditions.EmptyVehicleInitialPos=EmptyVehicleInitialPos;
InitialConditions.FullVehicleInitialPos=FullVehicleInitialPos; 

Passengers.Sources=Sources;
Passengers.Sinks=Sinks;
Passengers.Flows=FlowsIn;
Passengers.StartTimes=StartTimes;
Passengers.StarterSources=StarterSources;
Passengers.StarterSinks=StarterSinks;
Passengers.StarterFlows=StarterFlows;

Passengers.StarterSinks=[1];
Passengers.StarterSources={[1]};
Passengers.StarterFlows={[0]};

PowerNetwork.PowerGraphM=PowerGraph;
PowerNetwork.PowerLineCapM=PowerLineCap;
PowerNetwork.PowerLineReactanceM=PowerLineReactance;
PowerNetwork.PowerGensList=PowerGensList;
PowerNetwork.PowerGensMax=PowerGensMax;
PowerNetwork.PowerGensMin=PowerGensMin;
PowerNetwork.PowerCosts=PowerCosts;
PowerNetwork.PowerExtLoads=PowerExtLoads;
PowerNetwork.RoadToPowerMap=RoadToPowerMap;
PowerNetwork.v2g_efficiency=v2g_efficiency;
PowerNetwork.PowerRampUp=PowerRampUp;
PowerNetwork.PowerRampDown=PowerRampDown;

Flags.milpflag=milpflag;
Flags.congrelaxflag=congrelaxflag;
Flags.sourcerelaxflag=sourcerelaxflag;
Flags.cachedAeqflag=cachedAeqflag;
Flags.solverflag=solverflag;
Flags.minendchargerelaxflag=minendchargerelaxflag;
Flags.endreblocrelaxflag=endreblocrelaxflag;
Flags.powernetworkloadrelaxflag=powernetworkloadrelaxflag;
Flags.cyclicflag=cyclicflag;
%%
%[cplex_out,fval,exitflag,output,lambdas,dual_prices_ix,dual_charger_prices_ix]=TVPowerBalancedFlow_withpower_sinkbundle(Thor,RoadNetwork,PowerNetwork,InitialConditions,RebWeight,Passengers,Flags);
%%
[cplex_out,fval,exitflag,output,lambdas,dual_prices_ix,dual_charger_prices_ix]=TVPowerBalancedFlow_realtime(Thor,RoadNetwork,PowerNetwork,InitialConditions,RebWeight,Passengers,Flags);

%% Test sampling
[RebPaths,PaxChargeLevels]=TVPowerBalancedFlowSampler_realtime(cplex_out,Thor,RoadNetwork,PowerNetwork,InitialConditions,RebWeight,Passengers,Flags);

%% Output
DIAGNOSTIC_FLAG=0;
TVPowerBalancedFlowFinder_realtime;

for t=1:Thor
    fprintf('At time t=%d..\n',t)
    % Waiting passengers
    for k=1:MS
        for c=1:C 
            for ssi=1:length(StarterSources{k})
                if abs(cplex_out(FindStarterSourceChargetcks(t,c,k,ssi)))>1e-5
                    fprintf('%f waiting passenger(s) depart(s) from %d to %d at charge level %d\n',cplex_out(FindStarterSourceChargetcks(t,c,k,ssi)),StarterSources{k}(ssi),StarterSinks(k),c)
                end
            end
        end
        for c=1:C
            if cplex_out(FindStarterSinkChargetck(t,c,k))>=1e-5
                fprintf('%f waiting passenger(s) arrive(s) at %d at charge level %d\n',cplex_out(FindStarterSinkChargetck(t,c,k)),StarterSinks(k),c)
            end
        end
    end
    % Passengers
    for k=1:M
        for ssi=1:length(Sources{k})
            for c=1:C
                if abs(cplex_out(FindPaxSourceChargecks(c,k,ssi)))>=1e-5 && StartTimes{k}(ssi)==t
                    fprintf('%f passenger(s) departs from %d to %d at charge level %d\n',cplex_out(FindPaxSourceChargecks(c,k,ssi)),Sources{k}(ssi),Sinks(k),c)
                end
            end
        end
        for c=1:C
            if cplex_out(FindPaxSinkChargetck(t,c,k))>=1e-5
                fprintf('%f passenger(s) arrives at %d at charge level %d\n',cplex_out(FindPaxSinkChargetck(t,c,k)),Sinks(k),c)
            end
        end
    end
    % Reb. vehicles: travel
    for i=1:N
        for j=RoadGraph{i}
            for c=1:C
                if (abs(cplex_out(FindRoadLinkRtcij(t,c,i,j)))>=1e-5)
                    if j~=i
                        fprintf('%f reb. vehicle(s) drive from %d to %d at charge level %d\n',cplex_out(FindRoadLinkRtcij(t,c,i,j)),i,j,c)
                    else
                        %fprintf('%f reb. vehicle(s) stay at %d at charge level %d\n',cplex_out(FindRoadLinkRtcij(t,c,i,j)),i,c)
                    end
                end
            end
        end
    end
    % Reb. vehicles: charge and discharge
    for l=1:NumChargers
        for c=1:C
           if (abs(cplex_out(FindChargeLinkRtcl(t,c,l)))>=1e-5)
               fprintf('%f reb. vehicle(s) charge at %d at charge level %d\n',cplex_out(FindChargeLinkRtcl(t,c,l)),ChargersList(l),c);
           end
        end
    end
    for l=1:NumChargers
        for c=1:C
           if (abs(cplex_out(FindDischargeLinkRtcl(t,c,l)))>=1e-5)
               fprintf('%f reb. vehicle(s) discharges at %d at charge level %d\n',cplex_out(FindDischargeLinkRtcl(t,c,l)),ChargersList(l),c);
           end
        end
    end
    % Generators
    for i=1:NumGenerators
       if (abs(cplex_out(FindGeneratorti(t,i)))>=1e-5)
           fprintf('Generator %d produces %f energy\n',i,cplex_out(FindGeneratorti(t,i)))
       end
    end
end
fprintf('End locations:\n')
for i=1:N
    for c=1:C
        if abs(cplex_out(FindEndRebLocationci(c,i)))>=1e-5
            fprintf('%f vehicle(s) at %d with charge %d\n',cplex_out(FindEndRebLocationci(c,i)),i,c)
        end
    end
end
if sourcerelaxflag
    fprintf('Dropped pax:\n')
    for k=1:M
        for ssi=1:length(Sources{k})
            if abs(cplex_out(FindSourceRelaxks(k,ssi)))>=1e-5
                fprintf('Dropped %d pax from %d to %d at time %d\n',cplex_out(FindSourceRelaxks(k,ssi)),Sources{k}(ssi),Sinks(k),StartTimes{k}(ssi))
            end
        end
    end
    for k=1:MS
        for ssi=1:length(StarterSources{k})
            if abs(cplex_out(FindStarterSourceRelaxks(k,ssi)))>=1e-5
                fprintf('Dropped %d waiting pax from %d to %d\n',cplex_out(FindStarterSourceRelaxks(k,ssi)),StarterSources{k}(ssi),StarterSinks(k))
            end
        end
    end
end
if endreblocrelaxflag
    for i=1:N
        if abs(cplex_out(FindEndRebLocRelaxi(i)))>=1e-5
            fprintf('Relaxed end location constraint by %d at node %d\n',cplex_out(FindEndRebLocRelaxi(i)),i)
        end
    end
end
if powernetworkloadrelaxflag
    for t=1:Thor
        for i=1:NP
            if abs(cplex_out(FindPowerNetworkRelaxti(t,i)))>=1e-5
                fprintf('Relaxed power load by %d at time %d at node %d\n',cplex_out(FindPowerNetworkRelaxti(t,i)),t,i)
            end
        end
    end
end

% Tests:
% Move reb. vehicles and pick up pax at 1
% Require recharging before moving to pick up pax
% Multiple charging locations w. different prices
% Multiple charging locations w. constraints
% Background power loads
% Interconnected power network loads



