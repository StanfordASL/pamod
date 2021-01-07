function [] = PAMoD_optimize(input_scenario, output_file)

% Solves an instance of the P-AMoD optimization problem for a given input
% scenario.
% A scenario creator notebook for Dallas-Fort Worth is available in
% case_studies/Dallas_Fort-worth.
% Demo instances are available in the folder
%   case_studies/Dallas_Fort-Worth/scenario.
%
% The input scenario file contains the following variables
% (very roughly documented, see the scenario creator notebook
%  and the solver help for details):
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
    output_file = strcat(input_scenario, '_results');
end

% Include solvers
addpath(genpath('solvers'))

% These includes should be done outside, but as an additional line of
% defense...
%addpath(genpath('~/mosek/8/toolbox/r2014a'));
%addpath(genpath('/opt/ibm/ILOG/CPLEX_Studio128/cplex/matlab/x86-64_linux'))

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

fprintf('There are %d customers\n',sum(FlowsIn));

% disp('OVERRIDING vehicle costs:')
% BatteryCost = 15734.29;
% BatteryCycles = 1000; %More than 500, but not much more
% VehicleCostPerKm = 0.1601/1.6
% BatteryDepreciationPerUnitCharge = BatteryCost / (BatteryCycles*C)/2
%disp('Overriding v2g efficiency')
%v2g_efficiency=0;

fprintf('%d\n',num_nodes);
try
    num_vehicles>0;
catch
    num_vehicles=1e5;
end
%disp('OVERRIDE number of vehicles')
%num_vehicles=150000;

num_vehicles = double(num_vehicles)
BatteryDepreciationPerUnitCharge = double(BatteryDepreciationPerUnitCharge);

% Overrides
%num_vehicles=4*1e5;
%ChargerList=[1]
%ChargerSpeeds=[5]
%ChargerTimes=[1]
%ChargerCap=[0]
%MinEndCharge=C
%ChargeReqs=zeros(size(ChargeReqs));
%TravelTimes=TravelTimes>0;

%RoadToPowerMap=RoadToPowerMap(1)
%PowerToRoadMap=zeros(size(PowerToRoadMap));

%% Road data
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
RoadNetworkEmpty.ChargeUnitToPowerUnit=1;
RoadNetworkEmpty.ChargerCap=[];
RoadNetworkEmpty.MinEndCharge=1;
RoadNetworkEmpty.ValueOfTime=0;
RoadNetworkEmpty.VehicleCostPerKm=0;



% Power Data

PowerNetwork.PowerGraphM=PowerGraphFull;
PowerNetwork.PowerLineCapM=PowerLineCapFull;
PowerNetwork.PowerLineReactanceM=PowerLineReactanceFull;
PowerNetwork.PowerGensList=PowerGensList;
PowerNetwork.PowerGensMax=PowerGensMax';
PowerNetwork.PowerGensMin=PowerGensMin';
PowerNetwork.PowerCosts=PowerCosts';
PowerNetwork.PowerExtLoads=PowerExtLoads;
PowerNetwork.RoadToPowerMap=RoadToPowerMap;
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

InitialConditions.FullVehicleInitialPos=InitialPaxPos;
InitialConditions.EmptyVehicleInitialPos=InitialRebPos;

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

%% Non-RT solver
% First, solve for the power network only. If this is infesible, nothing we can do.
[cplex_out,fval,exitflag,output,lambdas,dual_prices_ix,dual_charger_prices_ix]=TVPowerBalancedFlow_withpower_sinkbundle(Thor,RoadNetworkEmpty,PowerNetwork,InitialConditions,RebWeight,PassengersEmpty,Flags);
% Next, solve the full problem.
[cplex_out,fval,exitflag,output,lambdas,dual_prices_ix,dual_charger_prices_ix]=TVPowerBalancedFlow_withpower_sinkbundle(Thor,RoadNetwork,PowerNetwork,InitialConditions,RebWeight,Passengers,Flags);

%[cplex_out_PO,fval_PO,exitflag_PO,output_PO,lambdas_PO,dual_prices_ix_PO]=TVPowerBalancedFlow_poweronly_multi(Thor,PowerNetwork,Flags);

DIAGNOSTIC_FLAG=0;
TVPowerBalancedFlowFinder_sinkbundle

%% Extract information

FullPowerPrices=reshape(lambdas.eqlin(dual_prices_ix>0),length(PowerNetwork.PowerGraphM),Thor);

%FullPowerPricesPO=reshape(lambdas_PO.eqlin(dual_prices_ix_PO>0),length(PowerNetwork.PowerGraphM),Thor);

numChargers = length(RoadNetwork.ChargersList);


% Match LMPs to the appropriate charger
LMP_ISO_ordered = zeros([numChargers,Thor]);

for t=1:Thor
    for l=1:numChargers 
        LMP_ISO_ordered(l,t)=FullPowerPrices(RoadToPowerMap(l),t);
    end
end
LMP_ISO_ordered;


%%
%ChargerPowerPrices=reshape(lambdas.eqlin(dual_charger_prices_ix>0),NumChargers,Thor);

ChargerPowerDemand=zeros(NumChargers,Thor);

for t=1:Thor
    for i=1:NP
        for l=1:NumChargers %Go through chargers in the road network
            if RoadToPowerMap(l)==i %If this node i in the power network corresponds to the charger
                for c=1:C
                    if c+ChargerSpeed(l)<=C
                        for k=1:M
                            % IF the charge link takes more than one unit time,
                            % we consider arcs that started at past times (up
                            % to t-ChargerTime+1),since they're still charging).
                            % In general, the amount of power per unit time
                            % delivered is ChargerSpeed/ChargerTime.
                            for deltat=0:ChargerTime(l)-1
                                if t-deltat>0 && t-deltat+ChargerTime(l)<=Thor
                                    ChargerPowerDemand(l,t)=ChargerPowerDemand(l,t)+cplex_out(FindChargeLinkPtckl(t-deltat,c,k,l))*ChargerSpeed(l)/ChargerTime(l)*ChargeUnitToPowerUnit;
                                end
                            end
                        end
                        for deltat=0:ChargerTime(l)-1
                            if t-deltat>0 && t-deltat+ChargerTime(l)<=Thor
                                ChargerPowerDemand(l,t)=ChargerPowerDemand(l,t)+cplex_out(FindChargeLinkRtcl(t-deltat,c,l))*ChargerSpeed(l)/ChargerTime(l)*ChargeUnitToPowerUnit;
                            end
                        end
                    end
                end
                for c=1:C
                    if c-ChargerSpeed(l)>=1
                        for k=1:M
                            for deltat=0:ChargerTime(l)-1
                                if t-deltat>0 && t-deltat+ChargerTime(l)<=Thor
                                    ChargerPowerDemand(l,t)=ChargerPowerDemand(l,t)-cplex_out(FindDischargeLinkPtckl(t-deltat,c,k,l))*v2g_efficiency*ChargerSpeed(l)/ChargerTime(l)*ChargeUnitToPowerUnit;
                                end
                            end
                        end

                        for deltat=0:ChargerTime(l)-1
                            if t-deltat>0 && t-deltat+ChargerTime(l)<=Thor
                                ChargerPowerDemand(l,t)=ChargerPowerDemand(l,t)-cplex_out(FindDischargeLinkRtcl(t-deltat,c,l))*v2g_efficiency*ChargerSpeed(l)/ChargerTime(l)*ChargeUnitToPowerUnit;
                            end
                        end
                    end
                end
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
            ArrivalTimeHist(tt)=ArrivalTimeHist(tt)+cplex_out(FindPaxSinkChargetck(tt,c,k));
        end
    end
end

% Average price of electricity
BusesPriceHist=mean(FullPowerPrices);

% Average price of electricity at chargers
%ChargersPriceHist=mean(ChargerPowerPrices);

% Average generation cost at chargers

GenTotCost=zeros(1,Thor);
GenProd=zeros(NumGenerators,Thor);
for tt=1:Thor
    for g=1:NumGenerators
        if cplex_out(FindGeneratorti(tt,g))>1e4
            tt
            g
            disp(cplex_out(FindGeneratorti(tt,g)))
        end
        GenTotCost(tt)=GenTotCost(tt)+cplex_out(FindGeneratorti(tt,g))*PowerCosts(tt,g);
        GenProd(g,tt)=cplex_out(FindGeneratorti(tt,g));
    end
end

% Charging vehicles
ChargingVehicleHist=zeros(1,Thor);
for tt=1:Thor
    for c=1:C
        for l=1:NumChargers
            for k=1:length(Sinks)
                ChargingVehicleHist(tt)=ChargingVehicleHist(tt)+cplex_out(FindChargeLinkPtckl(tt,c,k,l));
            end
            ChargingVehicleHist(tt)=ChargingVehicleHist(tt)+cplex_out(FindChargeLinkRtcl(tt,c,l));
        end
    end
end

% Discharging vehicles
DischargingVehicleHist=zeros(1,Thor);
for tt=1:Thor
    for c=1:C
        for l=1:NumChargers
            for k=1:length(Sinks)
                DischargingVehicleHist(tt)=DischargingVehicleHist(tt)+cplex_out(FindDischargeLinkPtckl(tt,c,k,l));
            end
            DischargingVehicleHist(tt)=DischargingVehicleHist(tt)+cplex_out(FindDischargeLinkRtcl(tt,c,l));
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
                                PaxVehicleHist(tt)=PaxVehicleHist(tt)+cplex_out(FindRoadLinkPtckij(tt-deltat,c,k,i,j));
                                PaxVehicleCharge(tt)=PaxVehicleCharge(tt)+c*cplex_out(FindRoadLinkPtckij(tt-deltat,c,k,i,j));
                            end
                            RebVehicleHist(tt)=RebVehicleHist(tt)+cplex_out(FindRoadLinkRtcij(tt-deltat,c,i,j));
                            RebVehicleCharge(tt)=RebVehicleCharge(tt)+c*cplex_out(FindRoadLinkRtcij(tt-deltat,c,i,j));
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


%% Sanity check on duals for link UBs and LBs
LinkUBidx=zeros(size(cplex_out));
for t=1:Thor
    for i=1:NP
        if ~isempty(PowerGraphFull{i})
            for j=1:length(PowerGraphFull{i})
                LinkUBidx(FindPowerLinktij(t,i,j))=1;
            end
        end
    end
end

LinkUBdual=LinkUBidx'*lambdas.upper;
LinkLBdual=LinkUBidx'*lambdas.lower;



% %%
% figure()
% 
%tplot=[5:0.5:14.5];
% 
% plot(tplot,DepTimeHist,'-.','LineWidth',2)
% hold all
% plot(tplot,ArrivalTimeHist,'--','LineWidth',2)
% hold all
% plot(tplot,ChargingVehicleHist,'LineWidth',2)
% hold all
% plot(tplot,DischargingVehicleHsaist,'LineWidth',2)
% hold all
% plot(tplot,PaxVehicleHist,'LineWidth',2)
% hold all
% plot(tplot,RebVehicleHist,'LineWidth',2)
% hold all
% 
% 
% legend('Departures','Arrivals','Charging vehicles','Discharging vehicles','Passenger Vehicles', 'Rebalancing Vehicles')
% disp('DO NOT USE, number of vehicles is computed incorrectly')

save(output_file)
