function [rebPaths, paxPaths,logs, paxChargeLevels] = PAMoDRebalancer(station,car,customer,LinkNumVehicles,LinkTime,RebOptions,RoadNetwork,PowerNetwork)

% Syntax: [rebPaths, paxPaths,logs, paxChargeLevels] = PAMoDRebalancer(station,car,customer,LinkNumVehicles,LinkTime,RebOptions,RoadNetwork,PowerNetwork)
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

LoadStateDefinitions;


numStations = length(station);
v=length(car);

t=RebOptions.t;
Thor=round(RebOptions.Thor_reb/RebOptions.predictionTimeStep);  %Time horizon in steps

predictionTimeStep=RebOptions.predictionTimeStep;           %Scale time in sim to time in optimizer
chargeProportion = RebOptions.chargeProportion;     %Scale charge in sim to charge in optimizer
chargeBias = RebOptions.chargeBias;                 %Scale charge in sim: baseline in sim which is not available to optimizer.
dt=RebOptions.dt;

RoadNetwork.TravelTimes=max(round(LinkTime/predictionTimeStep),LinkTime>0);
RoadNetwork.TravelDistance=RoadNetwork.LinkLength/1e3; %M to KM

%% We do not return this - it was used in the CDC paper.

paxPaths = cell(numStations,numStations);

%% From simulate_network, Ramon's work on time-varying MPC

% load passenger predictions into FlowsOut + FlowsIn
currentTime = ceil(t*dt/predictionTimeStep);

Passengers = predictPAMoDDemand(currentTime, Thor,predictionTimeStep,RebOptions);

%% load current trips/rebalancing into initial conditions
InitialConditions.EmptyVehicleInitialPosRT=zeros(Thor,RoadNetwork.N,RoadNetwork.C);

chargeinvec=zeros(v,1);
tinvec=zeros(v,1);

LowChargeVehiclesNo = zeros(4,1);

for i = 1:v
    carCharge=car(i).charge-chargeBias;
    % We compute these two anyway, just in case.
    timeToDest = 0;
    chargeToDest = 0;
    for pp=1:length(car(i).path)-1
        timeToDest = timeToDest + LinkTime(car(i).path(pp),car(i).path(pp+1));
        chargeToDest = chargeToDest + RoadNetwork.ChargeToTraverse(car(i).path(pp),car(i).path(pp+1));
    end
    
    timeToDest=timeToDest/(predictionTimeStep * dt);
    carCharge=carCharge/chargeProportion;
    chargeToDest=chargeToDest/chargeProportion;
    
    % if the car is in the process of pickup or dropoff
    if car(i).state == DRIVING_TO_DEST  || car(i).state == DRIVING_TO_STATION || car(i).state == REBALANCING

        tin = 1+ceil(timeToDest);
        assert(tin>0,'Weird, somehow a time-to-destination came out as zero in the optimizer')
        chargein=ceil(carCharge-chargeToDest);
        try
            assert(chargein>0,'Uh oh, some vehicle has zero or less charge in the optimizer')
        catch
           %fprintf('Uh oh, some vehicle has zero or less charge in the optimizer (state %d). Stopping for debugger', car(i).state)
           LowChargeVehiclesNo(1) = LowChargeVehiclesNo(1)+1;
           aa=2;
           %save(['BadChargeLevel_d2dest_',datestr(now)],'-v7.3')
           %pause
           continue
        end
        destnode = RoadNetwork.StationNodeID(car(i).dstation);
        %     elseif car(i).state == SELF_LOOP
        %         eta = car(i).time_left;
        %         tin = ceil(eta / (predictionStep * dt));
        %         if tin < Thor && tin > 0
        %             RoadNetwork.Starters(tin,car(i).dstation) = RoadNetwork.Starters(tin,car(i).dstation) + 1;
        %         elseif tin < Thor
        %             RoadNetwork.Starters(1,car(i).dstation) = RoadNetwork.Starters(1,car(i).dstation) + 1;
        %         end
    elseif car(i).state == IDLE
        tin=1;
        chargein=ceil(carCharge);
        try
            assert(chargein>0,'Uh oh, vehicle %d has zero charge in the optimizer (idle)',i)
        catch
           %fprintf('Uh oh, vehicle %d has zero or less charge in the optimizer (idle). Stopping for debugger',i)
           LowChargeVehiclesNo(2) = LowChargeVehiclesNo(2)+1;
           aa=2;
           %save(['BadChargeLevel_idle_',datestr(now)],'-v7.3')
           %pause
           continue
        end
        destnode = car(i).path(1);
        assert(car(i).ostation==car(i).dstation,'Idle car %d has different origin (%d) and dest. (%d) stations. Reset dstation in simulator.',i,car(i).ostation,car(i).dstation);
        assert(car(i).path(1)==RoadNetwork.StationNodeID(car(i).ostation),'Idle car %d has different location (node %d) and station (%d at node d)',i,car(i).path(1),car(i).ostation,RoadNetwork.StationNodeID(car(i).ostation));
    elseif car(i).state == DRIVING_TO_PICKUP
        eta1 = timeToDest;
        chargeToPax=carCharge-chargeToDest;
        cus = customer(car(i).passId);
        if cus.ostation == cus.dstation
            assert(0,'Customer with same origin and destination in the optimizer. Hey, this should not happen.')
            eta2 = 1;
        else
            eta2 = RoadNetwork.RouteTime(cus.ostation, cus.dstation); %RouteTime and RouteCharge are already in optimizer units
            chargeToDropoff=RoadNetwork.RouteCharge(cus.ostation, cus.dstation);
        end
        tin = 1+ceil(eta1+eta2);
        assert(tin>0,'Weird, somehow a time-to-destination came out as zero in the optimizer')
        chargein=ceil(chargeToPax-chargeToDropoff);
        try
            assert(chargein>0,'Uh oh, vehicle %d has zero charge in the optimizer (driving to pickup).',i)
        catch
            %fprintf('Uh oh, vehicle %d has zero charge in the optimizer (driving to pickup). Pausing\n',i)
            LowChargeVehiclesNo(3) = LowChargeVehiclesNo(3)+1;
            %save(['BadChargeLevel_d2pickup',datestr(now)],'-v7.3')
            aa=2;
            %pause;
            continue
        end
        destnode = RoadNetwork.StationNodeID(cus.dstation);
    elseif car(i).state == CHARGING | car(i).state == DISCHARGING
        tin=2;
        tmpcharger=find(RoadNetwork.ChargersList==car(i).dstation);
        assert(car(i).ostation==car(i).dstation,'Idle car %d has different origin (%d) and dest. (%d) stations. Reset dstation in simulator.',i,car(i).ostation,car(i).dstation);
        if tin == CHARGING
            chargein=ceil(carCharge+RoadNetwork.ChargerSpeedSim(tmpcharger)*predictionTimeStep);
        else
            chargein=ceil(carCharge-RoadNetwork.ChargerSpeedSim(tmpcharger)*predictionTimeStep);
        end
        try
            assert(chargein>0,'Uh oh, vehicle %d has zero (or less) charge in the optimizer (charging-discharging)',i)
        catch
            LowChargeVehiclesNo(4) = LowChargeVehiclesNo(4)+1;
            %fprintf('Uh oh, vehicle %d has zero (or less) charge in the optimizer (charging-discharging). Pausing\n',i)
            %save(['BadChargeLevel_d2chargedischarge',datestr(now)],'-v7.3')
            aa=2;
            %pause;
            continue
        end
        destnode = car(i).path(1);
    end
    
    if tin < Thor
        InitialConditions.EmptyVehicleInitialPosRT(tin,destnode,chargein) = InitialConditions.EmptyVehicleInitialPosRT(tin,car(i).dstation,chargein) + 1;
        tinvec(i)=tin;
        chargeinvec(i)=chargein;
    end
    
end

if sum(LowChargeVehiclesNo) >0
    fprintf('WARNING: %d vehicles have zero (or less) charge in the optimizer!\n %d driving to dest or station or rebalancing \n %d idle\n %d driving to pickup\n %d charging-discharging\n\n',sum(LowChargeVehiclesNo),LowChargeVehiclesNo(1),LowChargeVehiclesNo(2),LowChargeVehiclesNo(3),LowChargeVehiclesNo(4))
end

%% Final location of vehicles
InitialConditions.MinNumVehiclesAti=0.8*v/RoadNetwork.N*ones(RoadNetwork.N,1);


%% load waiting customers into FlowsOut

Passengers.StarterSinks=[1];
Passengers.StarterSources={[1]};
Passengers.StarterFlows={[0]};
for st=1:numStations
    for cu=1:length(station(st).custUnassigned)
        custInd = station(st).custUnassigned(cu);
        mysink=find(Passengers.StarterSinks == customer(custInd).dnode);
        if isempty(mysink) %If we have no such sink
            mysink=length(Passengers.StarterSinks)+1;
            Passengers.StarterSinks(mysink)=customer(custInd).dnode;
            Passengers.StarterSources{mysink}=[];
        end
            
        mysource=find(Passengers.StarterSources{mysink} == customer(custInd).onode);
        if isempty(mysource)
            mysource = length(Passengers.StarterSources{mysink})+1;
            Passengers.StarterSources{mysink}(mysource)=customer(custInd).onode;
            Passengers.StarterFlows{mysink}(mysource)=0;
        end
        Passengers.StarterFlows{mysink}(mysource)=Passengers.StarterFlows{mysink}(mysource)+1;
        
    end
end

%% Congestion
RoadNetwork.TVRoadCap=zeros(Thor,RoadNetwork.N,RoadNetwork.N);
for tt=1:Thor
    RoadNetwork.TVRoadCap(tt,:,:)=RoadNetwork.RoadCap-LinkNumVehicles;
end

%% Power network
DummyPowerNetwork=PowerNetwork;

DummyPowerNetwork.PowerGensMax=PowerNetwork.PowerGensMax(currentTime:currentTime+Thor-1,:);
DummyPowerNetwork.PowerGensMin=PowerNetwork.PowerGensMin(currentTime:currentTime+Thor-1,:);
DummyPowerNetwork.PowerCosts=PowerNetwork.PowerCosts(currentTime:currentTime+Thor-1,:);
DummyPowerNetwork.PowerExtLoads=PowerNetwork.PowerExtLoads(:,currentTime:currentTime+Thor-1);
DummyPowerNetwork.PowerRampUp = PowerNetwork.PowerRampUp(currentTime:currentTime+Thor-1,:);
DummyPowerNetwork.PowerRampDown= PowerNetwork.PowerRampDown(currentTime:currentTime+Thor-1,:);

%DummyPowerNetwork.PowerExtLoads=zeros(size(DummyPowerNetwork.PowerExtLoads));
%fprintf('\nREMEMBER TO RESET PowerGensMin\n')
%DummyPowerNetwork.PowerGensMin=zeros(size(DummyPowerNetwork.PowerGensMin));

% 

%%

% Flags,etc.
RebWeight=0;

Flags.milpflag=0;
Flags.congrelaxflag=1;
Flags.sourcerelaxflag=1;
Flags.cachedAeqflag=0;%1-RebOptions.firstRebalancerCall;
Flags.debugflag=0;
Flags.minendchargerelaxflag=1;
Flags.endreblocrelaxflag=1;
Flags.solverflag = 'MOSEK';
%% Call the optimizer



%DummyPowerNetwork=PowerNetwork;

DummyPassengers=Passengers;
%DummyPassengers.Sinks=[1];
%DummyPassengers.Sources={[2]};
%DummyPassengers.StartTimes={[2]};
%DummyPassengers.Flows={[1]};

[cplex_out,fval,exitflag,output,lambdas,dual_prices_ix,dual_charger_prices_ix,lb,solveTime]=TVPowerBalancedFlow_realtime(Thor,RoadNetwork,DummyPowerNetwork,InitialConditions,RebWeight,DummyPassengers,Flags);
try
    [rebPaths,paxChargeLevels]=TVPowerBalancedFlowSampler_realtime(cplex_out,Thor,RoadNetwork,DummyPowerNetwork,InitialConditions,RebWeight,DummyPassengers,Flags,lb);
    assert(exitflag>=0,'OPTIMIZER ERROR: failed to find a solution (exit flag: %d)\n',exitflag);
catch ME
    fprintf('OPTIMIZER ERROR: failed to find a solution (exit flag: %d) or trouble with the sampler. Enter debug mode (if you see this and are not in debug mode, reset break points...',exitflag)
    aa=2; %attach to debugger
    ME
    save(['OptimizerError_',datestr(now)],'-v7.3')
    %pause; %in case the debug flag is unser
end

%% Statistics, legacy code
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

logs.vownlog=vown;
logs.outstandingPaxlog=outstandingPax;
logs.custUnassigned=custUnassigned;
logs.solveTime=solveTime;

