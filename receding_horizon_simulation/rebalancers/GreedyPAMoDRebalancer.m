function [rebPaths, paxPaths,logs, paxChargeLevels] = GreedyPAMoDRebalancer(station,car,customer,LinkNumVehicles,LinkTime,RebOptions,RoadNetwork,PowerNetwork)

% Syntax: [rebPaths, paxPaths,logs, paxChargeLevels] = GreedyPAMoDRebalancer(station,car,customer,LinkNumVehicles,LinkTime,RebOptions,RoadNetwork,PowerNetwork)
% Optimizes charging and discharging in direct response to currenty-posted
% power prices. Uses PAMoDRebalancer internally.
%
% Inputs
% * station, car, customer, LinkNumVehicles, LinkTime, as defined in simulate_network_with_power
% * RebOptions, a struct containing options for the PAMoD rebalancer
%  - t
%  - Thor_reb
%  - predictionTimeStep
%  - chargeProportion
%  - chargeBias
%  - dt
%  - firstRebalancerCall
% * RoadNetwork and PowerNetwork as defined in simulate_network_with_power
%
% Outputs:
% * RebPaths, a N-by-C cell structure. RP{i,c} is a list of rebalancing
% paths departing from i at charge level c. The list may be empty. The list
% may contain charging and discharging paths.
% * PaxPaths, a N-by-N cell structure. PP{i,j} is a list of pax
% paths departing from i for j. The list may (and should, in this
% implementation) be empty.
% * logs, a struct containing logs used in the main optimizer:
%  - vownlog
%  - outstandingPaxlog
%  - custUnassigned
% * paxChargeLevels, a N by N cell structure. PCL{i,j} contains a list of
% charge levels for the vehicles that will pick up passengers going from i
% to j.

addpath(genpath('utilities'));

Thor_opt=size(PowerNetwork.PowerGensMax,1);%ceil(RebOptions.Tmax*RebOptions.dt/RebOptions.predictionTimeStep);


%% Build a dummy power network with the current power prices
N=length(RoadNetwork.RoadGraph);
GreedyPowerNetwork.VoLL = PowerNetwork.VoLL;
GreedyPowerNetwork.PowerGraphM=cell(N,1);
GreedyPowerNetwork.PowerLineCapM=cell(N,1);
GreedyPowerNetwork.PowerLineReactanceM=cell(N,1);
%PowerNetwork.PowerGraph=PowerGraph;
%PowerNetwork.PowerLineCap=PowerLineCap;
%PowerNetwork.PowerLineReactance=PowerLineReactance;
GreedyPowerNetwork.PowerGensList=1:1:N;
GreedyPowerNetwork.PowerGensMax=Inf*ones(Thor_opt,N);
GreedyPowerNetwork.PowerGensMin=-Inf*ones(Thor_opt,N);
if RebOptions.t == 1
    ChargersLoad = zeros(size(PowerNetwork.RoadToPowerMap));
    PreviousPowerGens = zeros(size(PowerNetwork.PowerGensList)); % Will be ignored anyway
    [GenLoads,PowerPrices,ChargerPrices] = TVPowerBalancedFlow_PowerNetworkPrices(RebOptions.t,RebOptions,RoadNetwork,LinkTime,PowerNetwork,PreviousPowerGens,RoadNetwork.ChargersList,ChargersLoad);    
else
    ChargerPrices = PowerNetwork.PreviousChargerPrices;
end
GreedyPowerNetwork.PowerCosts = repmat(ChargerPrices',[Thor_opt,1]);% ThorxN, with current prices. Could recompute or use existing ones - but how do we pass them?
%PowerCosts';

GreedyPowerNetwork.PowerExtLoads=zeros(N,Thor_opt);
GreedyPowerNetwork.RoadToPowerMap=1:1:N;
%PowerNetwork.PowerToRoadMap=PowerToRoadMap;
GreedyPowerNetwork.v2g_efficiency=PowerNetwork.v2g_efficiency;
GreedyPowerNetwork.PowerRampUp=Inf*ones(Thor_opt,N);
GreedyPowerNetwork.PowerRampDown=Inf*ones(Thor_opt,N);

%%
[rebPaths, paxPaths,logs, paxChargeLevels] = PAMoDRebalancer(station,car,customer,LinkNumVehicles,LinkTime,RebOptions,RoadNetwork,GreedyPowerNetwork);
