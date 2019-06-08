function [] = PAMoD_optimize_nocoord(input_scenario, output_file)

% Solves an instance of the uncoordinated P-AMoD problem for a given input
% scenario.
% A scenario creator notebook for Dallas-Fort Worth is available in
% case_studies/Dallas_Fort-worth.
% Demo instances are available in the folder
%   case_studies/Dallas_Fort-Worth/scenario.
%
% The input scenario file contains the following variables
% (very roughly documented, see the scenario creator notebook
% and the solver help for details):
%
%   Road network
%
% 'timeStepSize': the time step (in seconds)
% 'RoadGraph': the adjacency matrix of the road graph
% 'RoadCap': the road capacity of the road graph
% 'NodesLonLat': the location of the nodes in the road graph
% 'RefLonLat': a reference location used to compute x-y coords from lon-lat
% 'TravelTimes': travel times between road graph nodes
% 'SimTravelTimes':
% 'TravelDistance' : travel distance between nodes in the road graph
% 'ChargeReqs': charge required to travel between two nodes in the road
%               graph
% 'ChargerList': a list of nodes in the road graph that are equipped with
%                EV chargers
% 'ChargerSpeeds': the charging speed of the chargers, in units of
%                  power
% 'ChargerTimes': the time required to charge a vehicle by ChargerSpeeds[i]
% 'ChargerCaps': the maximum number of vehicles that a charger can
%                accommodate
% 'C': the number of discrete charge levels considered
% 'ChargeUnitToPowerUnit': the conversion factor from charge units to power
%                          units (base MVA in the power network) 
% 'BatteryDepreciationPerUnitCharge':
%
%    Customer Demand data
%
% 'Sources':    the start locations of customers
% 'Sinks':      the end locations of customers
% 'FlowsIn':    the customer rate for a given source-sink-start time
% 'StartTimes': the start time of a given customer flow
%
%   Power Network
%
% 'RoadToPowerMap' map from road network nodes to the closest power network
%                   node
% 'PowerToRoadMap': inverse of the map above
% 'PowerGraphFull': power network graph
% 'PowerLineCapFull': maximum line capacities
% 'PowerLineReactanceFull': line reactance
% 'PowerGensList': list of nodes in the power network that are generators
% 'PowerGensMax':  maximum power the generators can deliver as f(time)
% 'PowerGensMin':  minimum power the generators can deliver as f(time)
% 'PowerCosts':    generation economic costs
% 'PowerSocialCosts': generation social costs from pollution
% 'PowerExtLoads': exogenous loads in the power network
% 'ResidentialSolarID':
% 
%   Miscellaneous parameters
%   
% 'Thor': time horizon for the simulation
% 'BaseMVA': base MVA in the power network
% 'MinEndCharge': minimum end charge for the vehicles
% 'ValueOfTime': value of time
% 'VehicleCostPerKm': vehicle cost (maintenance, depreciation) per km
% 'v2g_efficiency' : efficiency of vehicle-to-grid
% 'PowerRampUp' : maximum ramp-up rate of generators
% 'PowerRampDown' : maximum ramp-down rate of generators
% 'num_vehicles' : number of vehicles

if nargin<2
    output_file = strcat(input_scenario, '_noncoord_results');
end

% Include solvers
addpath(genpath('solvers'))

% These includes should be done outside, but as an additional line of
% defense...
addpath(genpath('~/mosek/8/toolbox/r2014a'));
addpath(genpath('/opt/ibm/ILOG/CPLEX_Studio128/cplex/matlab/x86-64_linux'))

% Load data
load(input_scenario);

num_nodes = length(RoadGraph);
C = double(C);

%disp('Overriding minendcharge and roadcap and starttimes')
%MinEndCharge=0
%RoadCap=RoadCap*1000
%StartTimes=StartTimes-1
%ValueOfTime=12.2;

% initial distribution

init_distr = zeros(num_nodes,1);
for i=1:num_nodes
    init_distr(i) = sum(FlowsIn(Sources==i & StartTimes==1));
end
init_distr = init_distr / sum(init_distr);

% Modify source data. Note that for the TV version we'd want to keep unique
% (source, start time) pairs

Sources=Sources(FlowsIn>0);
Sinks=Sinks(FlowsIn>0);
StartTimes=StartTimes(FlowsIn>0);
FlowsIn=FlowsIn(FlowsIn>0);

CompactSinks=unique(Sinks);
CompactSinks=CompactSinks'; %We like column vectors
CompactSources=cell(length(CompactSinks),1);
CompactStartTimes=CompactSources;
CompactFlows=CompactSources;
for i=1:length(CompactSinks);
    CompactSources{i}=Sources(Sinks==CompactSinks(i));
    CompactStartTimes{i}=StartTimes(Sinks==CompactSinks(i));
    CompactFlows{i}=FlowsIn(Sinks==CompactSinks(i));
end

%Sources=CompactSources;
%Sinks=CompactSinks;
%StartTimes=CompactStartTimes;
%Flows=CompactFlows;

fprintf('There are %d customers\n',sum(FlowsIn));

% disp('OVERRIDING vehicle costs:')

% BatteryCost = 15734.29;
% BatteryCycles = 1000; %More than 500, but not much more
% VehicleCostPerKm = 0.1601/1.6
% BatteryDepreciationPerUnitCharge = BatteryCost / (BatteryCycles*C)/2


fprintf('%d\n',num_nodes);
try
    num_vehicles>0;
catch
    num_vehicles=1e5;
end
%disp('OVERRIDE number of vehicles')
%num_vehicles=550000;
num_vehicles = double(num_vehicles)
BatteryDepreciationPerUnitCharge = double(BatteryDepreciationPerUnitCharge);

% v2g_efficiency=0;
% disp('Changing V2G efficiency to inhibit v2g');


%%
% Road data
RoadNetwork.C=C;
RoadNetwork.RoadGraph=RoadGraph;
RoadNetwork.RoadCap=RoadCap;
RoadNetwork.TravelTimes=TravelTimes;
RoadNetwork.TravelDistance=TravelDistance;
RoadNetwork.ChargeToTraverse=ChargeReqs;
RoadNetwork.ChargersList=ChargerList
RoadNetwork.ChargerSpeed=ChargerSpeeds;
RoadNetwork.ChargerTime=ChargerTimes;
RoadNetwork.ChargeUnitToPowerUnit=ChargeUnitToPowerUnit;
RoadNetwork.ChargerCap=ChargerCaps;
RoadNetwork.MinEndCharge=MinEndCharge;
RoadNetwork.ValueOfTime=ValueOfTime;
RoadNetwork.VehicleCostPerKm=VehicleCostPerKm;
try
    RoadNetwork.BatteryDepreciationPerUnitCharge = BatteryDepreciationPerUnitCharge;
catch
    RoadNetwork.BatteryDepreciationPerUnitCharge = 0
end


assert(all(size(ChargerSpeeds) == size(ChargerList)))
assert(all(size(ChargerList) == size(ChargerTimes)))

RoadNetworkEmpty.C=C;
RoadNetworkEmpty.RoadGraph={};
RoadNetworkEmpty.RoadCap=[];
RoadNetworkEmpty.TravelTimes=[];
RoadNetworkEmpty.TravelDistance=[];
RoadNetworkEmpty.ChargeToTraverse=[];
RoadNetworkEmpty.ChargersList=[];
RoadNetworkEmpty.ChargerSpeed=[];
RoadNetworkEmpty.ChargerTime=[];
RoadNetworkEmpty.ChargeUnitToEnergyUnit=1;
RoadNetworkEmpty.ChargerCap=[];
RoadNetworkEmpty.MinEndCharge=1;
RoadNetworkEmpty.ValueOfTime=0;
RoadNetworkEmpty.VehicleCostPerKm=0;



% Power Data
%PowerGraph=cell(n_chargers,1);
%PowerLineCap=Inf(length(PowerGraph));

PowerNetwork.PowerGraphM=PowerGraphFull;
PowerNetwork.PowerLineCapM=PowerLineCapFull;
PowerNetwork.PowerLineReactanceM=PowerLineReactanceFull;
%PowerNetwork.PowerGraph=PowerGraph;
%PowerNetwork.PowerLineCap=PowerLineCap;
%PowerNetwork.PowerLineReactance=PowerLineReactance;
PowerNetwork.PowerGensList=PowerGensList;
PowerNetwork.PowerGensMax=PowerGensMax';
PowerNetwork.PowerGensMin=PowerGensMin';
PowerNetwork.PowerCosts=PowerCosts';
PowerNetwork.PowerExtLoads=PowerExtLoads;
PowerNetwork.RoadToPowerMap=RoadToPowerMap;
PowerNetwork.PowerToRoadMap=PowerToRoadMap;
PowerNetwork.v2g_efficiency=v2g_efficiency;
PowerNetwork.PowerRampUp=PowerRampUp;
PowerNetwork.PowerRampDown=PowerRampDown;

% Flow data
M=length(CompactSources);
Sources=CompactSources;
Sinks=CompactSinks;
FlowsIn=CompactFlows;
StartTimes=CompactStartTimes;

% The block below solves for the power network only
% Sources=[1]
% Sinks={[1]}
% FlowsIn={[0]}
% StartTimes=[1]
% num_vehicles=0

Passengers.Sources=Sources;
Passengers.Sinks=Sinks;
Passengers.Flows=FlowsIn;
Passengers.StartTimes=StartTimes;


PassengersEmpty.Sources={[1]};
PassengersEmpty.Sinks=[1];
PassengersEmpty.Flows={[0]};
PassengersEmpty.StartTimes={[1]};

% SET INITIAL CONDITIONS
InitialRebPos=zeros(num_nodes, C);
%InitialRebPos(CompactSources,C/2)=num_vehicles/length(CompactSources) *ones(length(CompactSources),1);
InitialRebPos(:,MinEndCharge)=num_vehicles * init_distr;
InitialPaxPos=zeros(M,num_nodes,C);

InitialRebPosEmpty=zeros(num_nodes, C);
InitialPaxPosEmpty=zeros(M,num_nodes,C);

InitialConditions.FullVehicleInitialPos=InitialPaxPos;
InitialConditions.EmptyVehicleInitialPos=InitialRebPos;

InitialConditionsEmpty.FullVehicleInitialPos=InitialPaxPosEmpty;
InitialConditionsEmpty.EmptyVehicleInitialPos=InitialRebPosEmpty;

% OPTIMIZATION CONFIG
milpflag=0;
congrelaxflag=0;
sourcerelaxflag=1;
cachedAeqflag=0;

Flags.milpflag=milpflag;
Flags.congrelaxflag=congrelaxflag;
Flags.sourcerelaxflag=sourcerelaxflag;
Flags.cachedAeqflag=cachedAeqflag;
Flags.solverflag='MOSEK';

RebWeight=0.2;

%% First we solve for the power network alone

[cplex_out_ISO,fval_ISO,exitflag_ISO,output_ISO,lambdas_ISO,dual_prices_ix_ISO,dual_charger_prices_ix_ISO]=TVPowerBalancedFlow_withpower_sinkbundle(Thor,RoadNetwork,PowerNetwork,InitialConditionsEmpty,RebWeight,PassengersEmpty,Flags);

% Extract prices
LMP_ISO = lambdas_ISO.eqlin(dual_prices_ix_ISO>0);

% Inequalities are organized by time first
NP = length(PowerGraphFull);
numChargers = length(RoadNetwork.ChargersList);
LMP_ISO = reshape(LMP_ISO,[NP,Thor]);

% Match LMPs to the appropriate charger
LMP_ISO_ordered = zeros([numChargers,Thor]);

for t=1:Thor
    for l=1:numChargers 
        LMP_ISO_ordered(l,t)=LMP_ISO(RoadToPowerMap(l),t);
    end
end
LMP_ISO_ordered;

% Extract power mix. We can't use the PowerFlowFinder because it expects specific names for the inputs,
%  so we rebuild a function from it.
MM=length(PassengersEmpty.Sinks); %Number of flows
NumGenerators=length(PowerNetwork.PowerGensList);
N=length(RoadGraph);
E=0;
for i=1:length(RoadGraph)
    RoadGraph{i}=sort(unique(RoadGraph{i}));
end
for i=1:N
    E=E+length(RoadGraph{i});
end

NumChargers=length(RoadNetwork.ChargersList);
NP=length(PowerNetwork.PowerGraphM);
EP=0;
for i=1:length(PowerNetwork.PowerGraphM)
    EP=EP+length(PowerNetwork.PowerGraphM{i});
end

TNS=1;%TotNumSources
FindGeneratorti_po = @ (t,i) (E*(MM+1)*(C) + 2*(MM+1)*NumChargers*C)*Thor + C*TNS + Thor*MM*C + C*N + (NumGenerators + NP + EP)*(t-1) + i; %Note that generators are referenced by their order, not their location in the graph. Similar to sources and sinks.

GenTotCost_ISO=zeros(1,Thor);
GenProd_ISO=zeros(NumGenerators,Thor);
for tt=1:Thor
    for g=1:NumGenerators
        FindGeneratorti_po(tt,g);
        GenTotCost_ISO(tt)=GenTotCost_ISO(tt)+cplex_out_ISO(FindGeneratorti_po(tt,g))*PowerCosts(g,tt);
        GenProd_ISO(g,tt)=cplex_out_ISO(FindGeneratorti_po(tt,g));
    end
end

%% Build reduced-size power graph

PowerGraphR=cell(1,numChargers);
PowerLineCapR=PowerGraphR;
PowerLineReactanceR=PowerGraphR;
PowerGensListR=1:1:numChargers;
PowerGensMaxR=Inf*ones(numChargers,Thor);
PowerGensMinR=-Inf*ones(numChargers,Thor);
PowerCostsR=LMP_ISO_ordered;
PowerExtLoadsR=zeros(NP,Thor);
RoadToPowerMapR= 1:1:N;

PowerNetworkR.PowerGraphM=PowerGraphR;
PowerNetworkR.PowerLineCapM=PowerLineCapR;
PowerNetworkR.PowerLineReactanceM=PowerLineReactanceR;
PowerNetworkR.PowerGensList=PowerGensListR;
PowerNetworkR.PowerGensMax=PowerGensMaxR';
PowerNetworkR.PowerGensMin=PowerGensMinR';
PowerNetworkR.PowerCosts=PowerCostsR';
PowerNetworkR.PowerExtLoads=PowerExtLoadsR;
PowerNetworkR.RoadToPowerMap=RoadToPowerMapR;
PowerNetworkR.v2g_efficiency=v2g_efficiency;

[cplex_out_TSO,fval_TSO,exitflag_TSO,output_TSO,lambdas_TSO,dual_prices_ix_TSO,dual_charger_prices_ix_TSO]=TVPowerBalancedFlow_withpower_sinkbundle(Thor,RoadNetwork,PowerNetworkR,InitialConditions,RebWeight,Passengers,Flags);

%% Recover loads from the solution

DIAGNOSTIC_FLAG=0;
TVPowerBalancedFlowFinder_sinkbundle %CAREFUL! In the Finder we use PowerGraph, not PowerGraphEmpty. However, we extract road variables, which come before power variables.

ChargerPowerDemand=zeros(NumChargers,Thor);

for t=1:Thor
    for l=1:NumChargers %Go through chargers in the road network
        i = RoadToPowerMap(l); %The node i in the power network corresponds to the charger
        for c=1:C
            for k=1:M
                % IF the charge link takes more than one unit time,
                % we consider arcs that started at past times (up
                % to t-ChargerTime+1),since they're still charging).
                % In general, the amount of power per unit time
                % delivered is ChargerSpeed/ChargerTime.
                for deltat=0:ChargerTime(l)-1
                    ChargerPowerDemand(l,t)=ChargerPowerDemand(l,t)+cplex_out_TSO(FindChargeLinkPtckl(t-deltat,c,k,l))*ChargerSpeed(l)/ChargerTime(l)*ChargeUnitToPowerUnit;
                end
            end
            for deltat=0:ChargerTime(l)-1
                ChargerPowerDemand(l,t)=ChargerPowerDemand(l,t)+cplex_out_TSO(FindChargeLinkRtcl(t-deltat,c,l))*ChargerSpeed(l)/ChargerTime(l)*ChargeUnitToPowerUnit;
            end
        end
        for c=1:C
            for k=1:M
                for deltat=0:ChargerTime(l)-1
                    ChargerPowerDemand(l,t)=ChargerPowerDemand(l,t)-cplex_out_TSO(FindDischargeLinkPtckl(t-deltat,c,k,l))*v2g_efficiency*ChargerSpeed(l)/ChargerTime(l)*ChargeUnitToPowerUnit;
                end
            end
                    
            for deltat=0:ChargerTime(l)-1
                ChargerPowerDemand(l,t)=ChargerPowerDemand(l,t)-cplex_out_TSO(FindDischargeLinkRtcl(t-deltat,c,l))*v2g_efficiency*ChargerSpeed(l)/ChargerTime(l)*ChargeUnitToPowerUnit;
            end
        end
    end     
end

PowerTotLoads=PowerExtLoads;
for t=1:Thor
    for l=1:NumChargers 
        PowerTotLoads(RoadToPowerMap(l),t)=PowerTotLoads(RoadToPowerMap(l),t)+ChargerPowerDemand(l,t);
    end
end

% Extract information about location of vehicles

%% Departure times
DepTimeHist=zeros(1,Thor);
for tt=1:Thor
    for ll=1:length(FlowsIn)
        for ssi=1:length(FlowsIn{ll})
            if StartTimes{ll}(ssi)==tt
                DepTimeHist(tt)=DepTimeHist(tt)+FlowsIn{ll}(ssi);
            end
        end
    end
end

% Arrival times
ArrivalTimeHist=zeros(1,Thor);
for tt=1:Thor
    for c=1:C
        for k=1:length(Sinks)
            ArrivalTimeHist(tt)=ArrivalTimeHist(tt)+cplex_out_TSO(FindPaxSinkChargetck(tt,c,k));
        end
    end
end


% Charging vehicles
ChargingVehicleHist=zeros(1,Thor);
for tt=1:Thor
    for c=1:C
        for l=1:NumChargers
            for k=1:length(Sinks)
                ChargingVehicleHist(tt)=ChargingVehicleHist(tt)+cplex_out_TSO(FindChargeLinkPtckl(tt,c,k,l));
            end
            ChargingVehicleHist(tt)=ChargingVehicleHist(tt)+cplex_out_TSO(FindChargeLinkRtcl(tt,c,l));
        end
    end
end

% Discharging vehicles
DischargingVehicleHist=zeros(1,Thor);
for tt=1:Thor
    for c=1:C
        for l=1:NumChargers
            for k=1:length(Sinks)
                DischargingVehicleHist(tt)=DischargingVehicleHist(tt)+cplex_out_TSO(FindDischargeLinkPtckl(tt,c,k,l));
            end
            DischargingVehicleHist(tt)=DischargingVehicleHist(tt)+cplex_out_TSO(FindDischargeLinkRtcl(tt,c,l));
        end
    end
end

% Moving vehicles
PaxVehicleHist=zeros(1,Thor);
RebVehicleHist=zeros(1,Thor);

PaxVehicleCharge=zeros(1,Thor);
RebVehicleCharge=zeros(1,Thor);
AllVehicleCharge=zeros(1,Thor);

for tt=1:Thor
    for c=1:C
        for i=1:num_nodes
            if ~isempty(RoadGraph{i})
                for j=RoadGraph{i}
                    for deltat=0:TravelTimes(i,j)-1 
                        if tt-deltat>0
                            for k=1:length(Sinks)
                                PaxVehicleHist(tt)=PaxVehicleHist(tt)+cplex_out_TSO(FindRoadLinkPtckij(tt-deltat,c,k,i,j));
                                PaxVehicleCharge(tt)=PaxVehicleCharge(tt)+c*cplex_out_TSO(FindRoadLinkPtckij(tt-deltat,c,k,i,j));
                            end
                            RebVehicleHist(tt)=RebVehicleHist(tt)+cplex_out_TSO(FindRoadLinkRtcij(tt-deltat,c,i,j));
                            RebVehicleCharge(tt)=RebVehicleCharge(tt)+c*cplex_out_TSO(FindRoadLinkRtcij(tt-deltat,c,i,j));
                        end
                    end
                end
            end
        end
    end
    tt
    AllVehicleCharge(tt)=PaxVehicleCharge(tt)+RebVehicleCharge(tt);
    PaxVehicleCharge(tt)=PaxVehicleCharge(tt)/PaxVehicleHist(tt);
    RebVehicleCharge(tt)=RebVehicleCharge(tt)/RebVehicleHist(tt);
    AllVehicleCharge(tt)=AllVehicleCharge(tt)/(PaxVehicleHist(tt)+RebVehicleHist(tt));
end

%% Resolve the power network problem and look at differences

PowerNetworkPost=PowerNetwork;
PowerNetworkPost.PowerExtLoads = PowerTotLoads;
Flags.powernetworkloadrelaxflag=1;

[cplex_out_post,fval_post,exitflag_post,output_post,lambdas_post,dual_prices_ix_post,dual_charger_prices_ix_post]=TVPowerBalancedFlow_withpower_sinkbundle(Thor,RoadNetwork,PowerNetworkPost,InitialConditions,RebWeight,PassengersEmpty,Flags);

% Extract prices
LMP_post = lambdas_post.eqlin(dual_prices_ix_post>0);

% Inequalities are organized by time first
NP = length(PowerGraphFull);
numChargers = length(RoadNetwork.ChargersList);
LMP_post = reshape(LMP_post,[NP,Thor]);

% Match LMPs to the appropriate charger
LMP_post_ordered = zeros([numChargers,Thor]);

for t=1:Thor
    for l=1:numChargers 
        LMP_post_ordered(l,t)=LMP_post(RoadToPowerMap(l),t);
    end
end

%LMP_post_ordered
%LMP_post_ordered-LMP_ISO_ordered

max(LMP_post);

% Extract load relaxations
DIAGNOSTIC_FLAG=0;

TempPax=Passengers;
Passengers=PassengersEmpty;

TVPowerBalancedFlowFinder_sinkbundle; %CAREFUL! In the Finder we use PowerGraph, not PowerGraphEmpty. However, we extract road variables, which come before power variables.
Passengers=TempPax;

LoadRelax=zeros(Thor,NP);
for t=1:Thor
    for i=1:NP
        LoadRelax(t,i)=cplex_out_post(FindPowerNetworkRelaxti(t,i));
    end
end
sum(sum(LoadRelax))


MM=1;
TNS=1;%TotNumSources
FindGeneratorti_po = @ (t,i) (E*(MM+1)*(C) + 2*(MM+1)*NumChargers*C)*Thor + C*TNS + Thor*MM*C + C*N + (NumGenerators + NP + EP)*(t-1) + i; %Note that generators are referenced by their order, not their location in the graph. Similar to sources and sinks.


GenTotCost_post=zeros(1,Thor);
GenProd_post=zeros(NumGenerators,Thor);
for tt=1:Thor
    for g=1:NumGenerators
        GenTotCost_post(tt)=GenTotCost_post(tt)+cplex_out_post(FindGeneratorti_po(tt,g))*PowerCosts(tt,g);
        GenProd_post(g,tt)=cplex_out_post(FindGeneratorti_po(tt,g));
    end
end



%% Save
save(output_file)