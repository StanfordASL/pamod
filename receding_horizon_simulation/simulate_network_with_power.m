function Stats = simulate_network_with_power(RoadNetwork, PowerNetwork, Trips, settings)

% Syntax:
% Stats = simulate_network_with_power(RoadNetwork, PowerNetwork, Trips, settings)
%
% This file has gone through a lot of iterations - it was previously known
% as simulate_network (Ramon ICRA 2018), road_network_sim (cs341 project,
% Federico RSS 2016), and perhaps with other names. In this implementation,
% we abstract the assignment of customers to vehicles in the function
% AssignCustomerToCar and the rebalancing problem in the function
% *Rebalancer (currently, PAMoDRebalancer and CongestionAwareRebalancer are
% implemented). We hope that this will foster code reuse and centralize
% debugging in the future.
% The file was originally developed by Rick Zhang and Federico Rossi
% (Federico likes structs; Rick does not). It was significantly improved
% and refactored by Ramon Iglesias. This version (with charging) was
% developed by Federico.
%
% Inputs
% RoadGraph: TODO
% trip_file: filename containing the trips
% Stations:


addpath(genpath('../solvers')); %Hack to get the functions in the root folder. Will remove.

addpath(genpath('utilities'));
addpath(genpath('rebalancers'));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% settings
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


dt = settings.dt;	% length of each time step
Tmax = settings.Tmax;	% simulation time
Treb = settings.Treb;       % how often do we rebalance
Thor_reb=settings.Thor_reb; % horizon of the rebalancing MPC algorithm
Tmincharge = settings.Tmincharge; %Minimum time for a charge-discharge action
findRouteMode = settings.findRouteMode;
%Tthresh = settings.Tthresh;     % at what time to start gathering data
forecastPeriods = settings.forecastPeriods; % number of periods to be used for forecasting (running average)
v = settings.v;     %number of vehicles
%These settings configure which algorithm is called. They will be
%modified for generality - F
UNPLANNEDPAXFLAG=settings.UNPLANNEDPAXFLAG;

ControllerMode=settings.ControllerMode;
if ~strcmp(ControllerMode,'GREEDY') && ~strcmp(ControllerMode,'PAMoD')
    disp('Controller mode not recognized! Switching to PAMoD. Press any key to continue')
    ControllerMode = 'PAMoD';
    pause
end

% How many customers have we loaded so far.
ccTmp = 1;
cc = 1; %keeps track of the customers already served by the system.

% Rebalancer bookkeeping
rebCount = 1; %Number of time steps since we last called the rebalancer.
firstRebalancerCall=1; %This is set to 0 after the rebalancer is called for the first time. It helps with caching

RebOptions.t=0; %This is changed in later calls.
RebOptions.Tmax=Tmax;
RebOptions.Thor_reb=Thor_reb;
RebOptions.dt=dt;
RebOptions.predictionTimeStep=settings.predictionTimeStep;
RebOptions.chargeProportion=1;
RebOptions.chargeBias=0;
RebOptions.firstRebalancerCall=firstRebalancerCall;
RebOptions.chargeBuffer=2; %we only assign a vehicle to a customer if, by the time they are done, they will have this charge left. This is separate from chargeBias and helps with the case where the vehicle picks a more energy-hungry but quicker route.

%% state definitions
LoadStateDefinitions;

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% data collection
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
avgWaitTimes = zeros(Tmax,1);
cumNumCustomers = zeros(Tmax,1);
numVehiclesBusy = zeros(Tmax, 1);
numVehiclesRebalancing = zeros(Tmax, 1);
numVehiclesNotRebalancing = zeros(Tmax, 1);
numCarsOnLink = cell(Tmax,1);
numVehiclesDtDest = zeros(Tmax, 1);
numVehiclesDtPick = zeros(Tmax, 1);
numVehiclesDtStat = zeros(Tmax, 1);
avgVehicleSpeeds = zeros(Tmax,1);
rebHist = zeros(Tmax,1);
numAstarRoutes=0;
numPlannedRoutes=0;
numComputedRebalancingRoutes=0;
vownlog = cell(Tmax,1);
vownlog_withpax = cell(Tmax,1);
waitingCustomerslog = cell(Tmax,1);
outstandingPaxlog = cell(Tmax,1);
linkSpeedLog = cell(Tmax,1);
vehicleStateLog = zeros(Tmax,numCarStates);
remainingPaxPaths=zeros(Tmax,1);
vehicleAvgCharge =zeros(Tmax,1);
GenLoadsLog = zeros(length(PowerNetwork.PowerGensList),Tmax);
PowerPricesLog = zeros(length(PowerNetwork.PowerGraphM),Tmax);
ChargerPricesLog = zeros(length(RoadNetwork.ChargersList),Tmax);
ChargersLoadLog =  zeros(length(RoadNetwork.ChargersList),Tmax);
MPCSolveTimeLog = zeros(Tmax,1);
PowerNetworkViolationBool = zeros(Tmax,1);
PowerNetworkViolationQUP = cell(Tmax,1);
PowerNetworkViolationQDN = cell(Tmax,1);


DropDeadCars = [];

% More data collection, only if we want to make a movie
if settings.VideoLog
    MovingCarsLoc=zeros(Tmax,v,2);
    %MovingCarsLink=zeros(Tmax,v,2);
 
    NumIdleCars = zeros(Tmax,RoadNetwork.N);
    NumMovingCars = zeros(Tmax,RoadNetwork.N,RoadNetwork.N);
    NumMovingCars2 = zeros(Tmax,RoadNetwork.N,RoadNetwork.N);
    IdleCarCharge = zeros(Tmax,RoadNetwork.N);
    MovingCarsCharge=zeros(Tmax,RoadNetwork.N,RoadNetwork.N);
    NumChargingCars = zeros(Tmax,RoadNetwork.N);
    NumDischargingCars = zeros(Tmax,RoadNetwork.N);
end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% variable definitions and unpacking
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
RoadGraph = RoadNetwork.RoadGraph;
RoadCap = RoadNetwork.RoadCap;
NodesLocation = RoadNetwork.NodesLocation;
LinkLength = RoadNetwork.LinkLength;
N = RoadNetwork.N;
LinkFreeFlowSpeed = RoadNetwork.LinkFreeFlowSpeed;
ChargersList = RoadNetwork.ChargersList;
ChargerSpeedSim = RoadNetwork.ChargerSpeedSim;

NumChargers = length(ChargersList);
ChargeToTraverse = RoadNetwork.ChargeToTraverse;
MaxCharge=RoadNetwork.C;

StationNodeID = RoadNetwork.StationNodeID;
StationLocation = RoadNetwork.StationLocation;

assert(sum(sum(xor(LinkLength>0,LinkFreeFlowSpeed>0)))==0,'The LinkLength matrix and the LinkFreeFlowSpeed matrix should have the same sparsity structure!')

%These will change during the simulation
LinkSpeed = LinkFreeFlowSpeed;
LinkTime = (LinkLength./LinkFreeFlowSpeed);
LinkTime(isnan(LinkTime))=0;
LinkNumVehicles = sparse(N,N);
numStations = length(StationNodeID);

%% more data collection
stateOfUnbalance = zeros(Tmax,1);
remainingRebPaths = zeros(numStations, Tmax);
idleVehicles = zeros(numStations, Tmax);
issuedRebPaths = zeros(numStations, Tmax);

% to keep track of arrivals
ArrivalsTracker=cell(Treb*forecastPeriods,1);

% this contains the passenger paths
paxPaths = cell(numStations,numStations);
paxChargeLevels = cell(numStations,numStations);
rebPaths = cell(numStations,RoadNetwork.C);

for i = numStations:-1:1
    station(i) = struct('id',i,'carIdle',[], 'custId',[], 'carOnRoad',[],...
        'custUnassigned',[],'waitTimes',[], 'arriveHour', [], 'node_id', StationNodeID(i));
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% load demand data
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
arrivalTimes = Trips.arrivalTimes;
try
    departureLocations= Trips.departureLocations;
    arrivalLocations  = Trips.arrivalLocations;
catch
    disp('Using NYC data, eh? The format for trip information has changed - this converter is deprecated and may be removed in a future version')
    departureLocations= Trips.MData(:,6:7)*1000;
    arrivalLocations  = Trips.MData(:,8:9)*1000;
end
predictFileName = Trips.predictFileName;

% track demand through time
expectedFlows = zeros(numStations,numStations);
custUnassigned = zeros(numStations,1);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% declare data structures for cars
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% struct for each car
stationCounter = 1;
fprintf('Building vehicles...')
for i = v:-1:1 %should make allocation slightly faster
    car(i) = struct('id',i,'passId', 0, 'dstation',stationCounter,'ostation',stationCounter,...
        'dpos', [], 'state', IDLE, 'pos', NodesLocation(station(stationCounter).node_id, :),...
        'direction',[], 'path', [StationNodeID(stationCounter)], 'dist_left', 0, 'charge', settings.initialCarCharge, 'desired_charge',settings.initialCarCharge);
    % update station data for this car
    station(stationCounter).carIdle = [station(stationCounter).carIdle, i];
    if stationCounter == numStations
        stationCounter = 1;
    else
        stationCounter = stationCounter + 1;
    end
end

% disp('Experiments with charge...')
% car(1).charge=0;
% keyboard

fprintf('done.\n')

%customer(1)=struct('opos',-1, 'dpos', -1,...
%                'onode', -1, 'dnode', -1, 'ostation',-1, 'dstation', tmpStations(2), ...
%                'waitTime',0,'serviceTime',0,'pickedup',0,'delivered',0);;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Begin Simulation: main loop
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

try %Wrapped in a try-catch block so that, if anything goes wrong, we get access to the internal state
    
    for t = 1:Tmax-1
        % Note that t is the timestep in the *simulation*. Clock time is t*dt
        %t
        %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Logging and print some statistics:
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        numDeliveredR = 0;
        numPickedUpR = 0;
        totalServiceTimeR = 0;
        totalWaitTimeR = 0;
        totalWaitTimeR_pu = 0;
        for i = 1:cc-1
            if customer(i).delivered
                numDeliveredR = numDeliveredR + 1;
                totalServiceTimeR = totalServiceTimeR + customer(i).serviceTime;
                totalWaitTimeR = totalWaitTimeR + customer(i).waitTime;
            end
            if customer(i).pickedup
                numPickedUpR=numPickedUpR+1;
                totalWaitTimeR_pu = totalWaitTimeR_pu + customer(i).waitTime;
            end
        end
        meanServiceTimeR = totalServiceTimeR/numDeliveredR;
        meanWaitTimeR = totalWaitTimeR/numDeliveredR;
        meanWaitTimeR_pu = totalWaitTimeR_pu/numPickedUpR;
        vehicleAvgCharge(t)=0;
        % collect vehicle state
        for i = 1:v
            vehicleAvgCharge(t)=vehicleAvgCharge(t)+car(i).charge;
        end
        vehicleAvgCharge(t)=vehicleAvgCharge(t)/length(car);
        DispHour = floor(t*dt/3600);
        DispMin  = floor(t*dt/60)-60*DispHour;
        DispSec  = t*dt-60*DispMin-60*60*DispHour;
        fprintf('%s: t=%d (%0.2d:%0.2d:%0.2d), avg. travel time so far %f s, wait time %f s (%f s incl. pax. in transit), picked up pax %d, delivered pax %d, unassigned pax (as of last reb) %d, avg. charge level %d\n',...
            ControllerMode,t,DispHour,DispMin,DispSec,meanServiceTimeR,meanWaitTimeR,meanWaitTimeR_pu,numPickedUpR,numDeliveredR,sum(custUnassigned),vehicleAvgCharge(t))
        
        % update LinkTime variable
        % go through all the links (i,j)
        for i = 1:N
            for j = RoadGraph{i}
                if j~=i
                    [tmpLinkTime, tmpLinkSpeed] = getSpeed(LinkNumVehicles(i,j), RoadCap(i,j), LinkFreeFlowSpeed(i,j), LinkLength(i,j));
                else
                    tmpLinkTime=dt;
                    tmpLinkSpeed=Inf;
                end
                LinkTime(i,j) = tmpLinkTime;
                LinkSpeed(i,j) = tmpLinkSpeed;
            end
        end
        
        %Logging
        linkSpeedLog{t}=LinkSpeed;
        
        %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % vehicle state transitions:
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        for i = 1:v
            if car(i).state ~= IDLE && car(i).state ~= CHARGING && car(i).state ~= DISCHARGING  && length(car(i).path) == 2 % if it's on the last leg of its trip
                distToDest = norm(car(i).dpos - car(i).pos,2);
                if distToDest < LinkSpeed(car(i).path(1), car(i).path(2))*dt
                    % remove car from link
                    LinkNumVehicles(car(i).path(1), car(i).path(2)) = LinkNumVehicles(car(i).path(1), car(i).path(2)) - 1;
                    % shrink path to just final destination
                    car(i).path = car(i).path(2);
                    if car(i).state == DRIVING_TO_PICKUP
                        % pick up customer
                        car(i).state = DRIVING_TO_DEST;
                        car(i).pos = car(i).dpos;
                        car(i).dpos = customer(car(i).passId).dpos;
                        customer(car(i).passId).pickedup = 1;
                        %%%%%%%%%%%%%%%%%%% TODO (RI,3/16): this should take a path from a set of precomputed paths!
                        %disp('TURNED OFF PRECOMPUTED PAX ROUTES')
                        if ~isempty(paxPaths{customer(car(i).passId).ostation, customer(car(i).passId).dstation})
                            
                            numPlannedRoutes=numPlannedRoutes+1;
                            nominalPath = paxPaths{customer(car(i).passId).ostation, customer(car(i).passId).dstation}{1};
                            realPath = findFinalPath(nominalPath, customer(car(i).passId).onode, customer(car(i).passId).dnode, NodesLocation, LinkTime);
                            % remove the path from our available paths
                            if length(paxPaths{customer(car(i).passId).ostation, customer(car(i).passId).dstation})>1
                                paxPaths{customer(car(i).passId).ostation, customer(car(i).passId).dstation} = paxPaths{customer(car(i).passId).ostation, customer(car(i).passId).dstation}(2:end);
                            else
                                paxPaths{customer(car(i).passId).ostation, customer(car(i).passId).dstation}=[];
                            end
                        else
                            numAstarRoutes=numAstarRoutes+1;
                            realPath = findRoute(car(i).path(1), customer(car(i).passId).dnode, LinkTime,RoadGraph,NodesLocation,findRouteMode,RoadNetwork.Routes);
                        end
                        car(i).path = realPath;
                        
                        LinkNumVehicles(car(i).path(1), car(i).path(2)) = LinkNumVehicles(car(i).path(1), car(i).path(2)) + 1;
                    elseif car(i).state == DRIVING_TO_DEST
                        % drop off customer
                        car(i).state = DRIVING_TO_STATION;
                        car(i).pos = customer(car(i).passId).dpos;
                        car(i).dpos = StationLocation(customer(car(i).passId).dstation, :);
                        % route car to station
                        car(i).path = findRoute(car(i).path(1), station(customer(car(i).passId).dstation).node_id, LinkTime,RoadGraph,NodesLocation,findRouteMode,RoadNetwork.Routes);
                        tmpindex = find(station(customer(car(i).passId).ostation).custId == car(i).passId);
                        % remove customer from the origin station
                        station(customer(car(i).passId).ostation).custId = [station(customer(car(i).passId).ostation).custId(1:tmpindex-1), station(customer(car(i).passId).ostation).custId(tmpindex+1:end)];
                        % car is now free to receive assignments from station
                        
                        customer(car(i).passId).delivered = 1;
                        car(i).passId = 0;
                        
                        % if the destination was already a station
                        if length(car(i).path) == 1
                            car(i).state = IDLE;
                            car(i).ostation = car(i).dstation;
                            station(car(i).ostation).carIdle = [station(car(i).ostation).carIdle, car(i).id];
                            try
                                assert(NodesLocation(StationNodeID(car(i).ostation),1)== car(i).pos(1) & NodesLocation(StationNodeID(car(i).ostation),2)== car(i).pos(2),'ERROR: car is not where it is supposed to be.')
                            catch
                                aa=2
                                pause
                            end
                        else
                            station(car(i).dstation).carOnRoad = [station(car(i).dstation).carOnRoad, car(i).id];
                            LinkNumVehicles(car(i).path(1), car(i).path(2)) = LinkNumVehicles(car(i).path(1), car(i).path(2)) + 1;
                        end
                        
                    elseif car(i).state == DRIVING_TO_STATION
                        % arrived back to station
                        car(i).state = IDLE;
                        car(i).pos = car(i).dpos;
                        car(i).ostation = car(i).dstation;
                        % add car to idle car list
                        station(car(i).ostation).carIdle = [station(car(i).ostation).carIdle, car(i).id];
                        tmpindex = find(station(car(i).ostation).carOnRoad == car(i).id);
                        % remove car from caronroad list
                        station(car(i).ostation).carOnRoad = [station(car(i).ostation).carOnRoad(1:tmpindex-1), station(car(i).ostation).carOnRoad(tmpindex+1:end)];
                        
                    elseif car(i).state == REBALANCING
                        % finished rebalancing to station
                        car(i).state = IDLE;
                        car(i).pos = car(i).dpos;
                        car(i).ostation = car(i).dstation;
                        % add car to idle car list
                        station(car(i).ostation).carIdle = [station(car(i).ostation).carIdle, car(i).id];
                        tmpindex = find(station(car(i).ostation).carOnRoad == car(i).id);
                        % remove car from caronroad list
                        if ~isempty(tmpindex)
                            station(car(i).ostation).carOnRoad = [station(car(i).ostation).carOnRoad(1:tmpindex-1), station(car(i).ostation).carOnRoad(tmpindex+1:end)];
                        end
                    elseif car(i).state == LIMP_HOME
                        car(i).state = CHARGING;
                        car(i).dstation = car(i).ostation;
                        car(i).pos = car(i).dpos;
                        car(i).desired_charge = 1;
                    end
                end
            end
            if car(i).state == CHARGING || car(i).state == DISCHARGING
                assert(length(car(i).path)==1 && car(i).pos(1) == NodesLocation(car(i).path(1),1) && car(i).pos(2) == NodesLocation(car(i).path(1),2) && ~isempty(find(ChargersList==car(i).path(1), 1)),'Vehicle %d wants to charge/discharge but it is not at a charger (at node %d)\n',i,car(i).pos)
                %the vehicle's charge level is close to the desired charge level
                tmpcharger=find(ChargersList==car(i).path(1));
                if abs(car(i).charge-car(i).desired_charge)<=ChargerSpeedSim(tmpcharger)*dt %If the vehicle is going to be done charging by the end of the time step
                    car(i).charge=car(i).desired_charge;
                    car(i).state = IDLE;
                    %Return car to the carIdle pool
                    assert(car(i).ostation==car(i).dstation,'ERROR with charging: car %d has different ostation and dstation',i);
                    station(car(i).ostation).carIdle = [station(car(i).ostation).carIdle, car(i).id];
                end
            end
        end
        
        %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % new customer arrivals
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        tempArrivals = zeros(numStations, numStations);
        while ccTmp<=length(arrivalTimes) && arrivalTimes(ccTmp) < t
            % to keep track of forecast demand
            %customer arrival and destination locations
            tmpCust = [departureLocations(ccTmp,:); arrivalLocations(ccTmp,:)];
            % find the nearest nodes
            tmpNodes = dsearchn(NodesLocation, tmpCust);
            if tmpNodes(1) ~= tmpNodes(2) % Check that start/end nodes are not the same
                % find the stations
                tmpStations = dsearchn(StationLocation, tmpCust);
                % make customer structure
                customer(cc) = struct('opos',NodesLocation(tmpNodes(1),:), 'dpos', NodesLocation(tmpNodes(2),:),...
                    'onode', tmpNodes(1), 'dnode', tmpNodes(2), 'ostation',tmpStations(1), 'dstation', tmpStations(2), ...
                    'waitTime',0,'serviceTime',0,'pickedup',0,'delivered',0);
                % add this customer to the station
                station(customer(cc).ostation).custId = [station(customer(cc).ostation).custId  cc];
                station(customer(cc).ostation).custUnassigned = [station(customer(cc).ostation).custUnassigned  cc];
                % add to the arrival tracker
                tempArrivals(tmpStations(1), tmpStations(2)) = tempArrivals(tmpStations(1), tmpStations(2)) + 1;
                cc = cc + 1;
                ccTmp = ccTmp + 1;
            else
                ccTmp = ccTmp + 1;
            end
            
            % update forecast
            
        end
        ArrivalsTracker{mod(t-1,Treb*forecastPeriods)+1}=tempArrivals; % %circular buffer
        
        %expectedFlows = ((expectedFlows * (forecastPeriods * Thor - 1) + tempArrivals)/(forecastPeriods * Thor));
        
        
        %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % assign vehicles to new customers
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        % Used: stations (list of unassigned customers, idle cars);%
        %       Customers (ostation, dstation), position
        %       StationLocation
        %       Car (pos)
        %       RoadNetwork.LinkTime (for shortest path)
        %       RoadNetwork.NodesLocation
        %       PaxPaths (pre-cached by rebalancer)
        %
        %
        %       Returns: cars, pax, stations (no idle car), LinkNumVehicles
        
        %TODO: refactor. A matching function should take in Stations,
        %Customers, and Cars, and return a list of custIndVec,
        %assignedCarIDVec, and ToReassign (one per station).
        % Note that, since routing is station-wise, we could in principle have
        % the function do the matching within each station, simplifying things.
        %
        % The code below should then process things to assign
        %the cars to the customers. Will come back to this.
        
        for i = 1:numStations
            % as long as there are cars left or customers waiting
            ToReassign=[];
            while ~isempty(station(i).custUnassigned) && (~isempty(station(i).carIdle) || ~isempty(station(i).carOnRoad))
                custInd = station(i).custUnassigned(1);
                % get distance from customer
                %if 1 %here check whether we want to dispatch a vehicle to the customer. Depending on our staggering strategy, we may not. if length(paxPaths{customer(custInd).ostation, customer(custInd).dstation}) > 0
                if (~isempty(paxChargeLevels{customer(custInd).ostation, customer(custInd).dstation}) | ~isempty(paxPaths{customer(custInd).ostation, customer(custInd).dstation}) | UNPLANNEDPAXFLAG)
                    
                    % Planned, charge-aware
                    if ~isempty(paxChargeLevels{customer(custInd).ostation, customer(custInd).dstation})
                        % This pax is planned. let's look for a vehicle with
                        % the right charge level
                        tmpChargeLevel = paxChargeLevels{customer(custInd).ostation, customer(custInd).dstation}(1);
                        assignedCarID=-1;
                        shortestDistance=Inf;
                        chargeLevels=zeros(size(station(i).carIdle));
                        chargeLevelCt=1;
                        for vv=station(i).carIdle
                            assert(car(vv).state == IDLE,'ERROR: non-idle car in carIdle (car %d has state %d)',vv,car(vv).state)
                            carCharge=ceil((car(vv).charge-RebOptions.chargeBias)/RebOptions.chargeProportion); % compute discretized charge level (use same data given to controller)
                            chargeLevels(chargeLevelCt)=carCharge;
                            chargeLevelCt=chargeLevelCt+1;
                            if carCharge==tmpChargeLevel
                                assignedCarID=vv;
                                shortestDistance = norm(customer(custInd).opos - car(vv).pos);
                                break
                            end
                        end
                        
                        %Now look among moving cars
                        candidateMovingCars=[];
                        for vv=station(i).carOnRoad
                            carCharge=ceil((car(vv).charge-RoadNetwork.RouteCharge(car(vv).path(1),StationNodeID(station(i).id))-RebOptions.chargeBias)/RebOptions.chargeProportion); % compute discretized charge level (use same data given to controller)
                            if carCharge==tmpChargeLevel
                                candidateMovingCars=[candidateMovingCars, vv];
                            end
                        end
                        
                        carOnRoadIndex = 0;
                        for vv = candidateMovingCars
                            distToCar = norm(customer(custInd).opos - car(vv).pos);
                            if distToCar < shortestDistance
                                shortestDistance = distToCar;
                                assignedCarID = vv;
                                carOnRoadIndex = find(assignedCarID==station(i).carOnRoad,1);
                            end
                        end
                        
                        if assignedCarID<=0 %,'Weird: we have a planned trip from station %d to station %d at charge %d, but no vehicles!',customer(custInd).ostation,customer(custInd).dstation,tmpChargeLevel);
                            
                            % Looks like someone stole our car! Let's fall back
                            % to nearest-neighbor assignment
                            %aa=2 %for debugger
                            %pause
                            [assignedCarID,carOnRoadIndex]=AssignCustomerToCar(customer(custInd),station(i),StationLocation(i,:),car,RoadNetwork,RebOptions);
                            if assignedCarID==-1 %Nobody can pick up this customer - not enough charge!
                                ToReassign=[ToReassign, station(i).custUnassigned(1)];
                                station(i).custUnassigned = station(i).custUnassigned(2:end);
                                continue
                            end
                        else
                            %Remove charge level from list
                            paxChargeLevels{customer(custInd).ostation, customer(custInd).dstation}=paxChargeLevels{customer(custInd).ostation, customer(custInd).dstation}(2:end);
                        end
                    else
                        % Charge-unaware
                        % congestion-aware assignment: look up required charge
                        % level to go from ostation to dstation. If no reported
                        % level, TODO. IF reported level, look for car with that
                        % level (if there is no such car, report error, that
                        % shouldn't happen).
                        [assignedCarID,carOnRoadIndex]=AssignCustomerToCar(customer(custInd),station(i),StationLocation(i,:),car,RoadNetwork,RebOptions);
                        if assignedCarID==-1 %Nobody can pick up this customer - not enough charge!
                            ToReassign=[ToReassign, station(i).custUnassigned(1)];
                            station(i).custUnassigned = station(i).custUnassigned(2:end);
                            continue
                        end
                    end
                    
                    
                    % Now the car is assigned
                    
                    % assign route to the vehicle from its location to the passenger
                    %assignedCarID
                    %car(assignedCarID).passId = custInd;
                    tmpPath = car(assignedCarID).path;
                    tmpState= car(assignedCarID).state;
                    car(assignedCarID).path = findRoute(tmpPath(1), customer(custInd).onode, LinkTime,RoadGraph,NodesLocation,findRouteMode,RoadNetwork.Routes);
                    
                    alreadyAtPickup = 0;
                    if length(tmpPath) == 1 && length(car(assignedCarID).path) == 1
                        % vehicle is not initially in motion (at a station)
                        % go straight to routing delivery
                        alreadyAtPickup = 1;
                        car(assignedCarID).state = DRIVING_TO_DEST;
                        car(assignedCarID).dpos = customer(custInd).dpos;
                        customer(custInd).pickedup = 1;
                        
                        %%%%%%%%%%%%%%%%%%% TODO (RI,3/16): this should take a path from a set of precomputed paths!
                        % route the new path
                        if length(paxPaths{customer(custInd).ostation, customer(custInd).dstation}) > 0
                            numPlannedRoutes=numPlannedRoutes+1;
                            nominalPath = paxPaths{customer(custInd).ostation, customer(custInd).dstation}{1};
                            realPath = findFinalPath(nominalPath, customer(custInd).onode, customer(custInd).dnode, NodesLocation, LinkTime);
                            % remove the path from our available paths
                            if length(paxPaths{customer(custInd).ostation, customer(custInd).dstation})>1
                                paxPaths{customer(custInd).ostation, customer(custInd).dstation} = paxPaths{customer(custInd).ostation, customer(custInd).dstation}(2:end);
                            else
                                paxPaths{customer(custInd).ostation, customer(custInd).dstation}=[];
                            end
                        else
                            numAstarRoutes=numAstarRoutes+1;
                            realPath = findRoute(tmpPath(1), customer(custInd).dnode, LinkTime,RoadGraph,NodesLocation,findRouteMode,RoadNetwork.Routes);
                        end
                        car(assignedCarID).path = realPath;
                    else
                        % vehicle is in motion.
                        if length(car(assignedCarID).path) == 1
                            % the vehicle must find its way back
                            car(assignedCarID).path = [tmpPath(1) findRoute(tmpPath(2), customer(custInd).onode, LinkTime,RoadGraph,NodesLocation,findRouteMode,RoadNetwork.Routes)];
                        elseif length(tmpPath) > 1
                            if car(assignedCarID).path(2) ~= tmpPath(2)
                                % go a different way - again you have to find your way around
                                car(assignedCarID).path = [tmpPath(1) findRoute(tmpPath(2), customer(custInd).onode, LinkTime,RoadGraph,NodesLocation,findRouteMode,RoadNetwork.Routes)];
                            end
                        end
                    end
                    if length(car(assignedCarID).path) > 1 && tmpState == IDLE
                        LinkNumVehicles(car(assignedCarID).path(1), car(assignedCarID).path(2)) = LinkNumVehicles(car(assignedCarID).path(1), car(assignedCarID).path(2))+1;
                    end
                    
                    % remove assigned car from station
                    %TODO WRONG
                    
                    if carOnRoadIndex<=0 % we picked an idle car
                        station(i).carIdle = station(i).carIdle(station(i).carIdle~=assignedCarID);
                    else
                        station(i).carOnRoad = [station(i).carOnRoad(1:carOnRoadIndex-1), station(i).carOnRoad(carOnRoadIndex+1:end)];
                    end
                    
                    %if ~isempty(station(i).carIdle) && assignedCarID == station(i).carIdle(1)
                    %    station(i).carIdle = station(i).carIdle(2:end);
                    %else
                    %    station(i).carOnRoad = [station(i).carOnRoad(1:carOnRoadIndex-1), station(i).carOnRoad(carOnRoadIndex+1:end)];
                    %end
                    %/TODO WRONG
                    
                    
                    % assign the car to the customer
                    car(assignedCarID).dstation = customer(custInd).dstation;
                    car(assignedCarID).ostation = i;
                    if ~alreadyAtPickup
                        car(assignedCarID).state = DRIVING_TO_PICKUP;
                        car(assignedCarID).dpos = customer(custInd).opos;
                        valid = 1;
                        if length(car(assignedCarID).path) == 1
                            continue
                        end
                        for q=1:(length(car(assignedCarID).path)-1)
                            if ~any(RoadGraph{car(assignedCarID).path(q)} == car(assignedCarID).path(q+1))
                                valid = 0;
                            end
                        end
                        if ~valid
                            assert(-1,'INVALID PATH FOR PICKUP')
                        end
                    end
                    car(assignedCarID).passId = custInd;
                    
                    % remove unassigned customer from station
                    station(i).custUnassigned = station(i).custUnassigned(2:end);
                else
                    ToReassign=[ToReassign, station(i).custUnassigned(1)];
                    station(i).custUnassigned = station(i).custUnassigned(2:end);
                end
            end
            station(i).custUnassigned=[ToReassign,station(i).custUnassigned];
        end
        
        %Saving the number of unused paxPaths
        
        for ii=1:size(paxPaths,1)
            for jj=1:size(paxPaths,2)
                remainingPaxPaths(t)=remainingPaxPaths(t)+length(paxPaths{ii,jj});
            end
        end
        
        %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % call rebalancing
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        if rebCount >= Treb
            rebCount = 1;
            RebOptions.t=t;
            RebOptions.Thor_reb=Thor_reb;
            RebOptions.dt=dt;
            RebOptions.predictionTimeStep=settings.predictionTimeStep;
            RebOptions.chargeProportion=1;
            RebOptions.chargeBias=0;
            RebOptions.firstRebalancerCall=firstRebalancerCall;
            RebOptions.predictFileName = predictFileName;
            RebOptions.customerStdDev = settings.customerStdDev;
            
            %[rebPaths, paxPaths,MPCLogs] = CongestionAwareRebalancer(station,car,customer,LinkNumVehicles,LinkTime,RebOptions,RoadNetwork,PowerNetwork);
            if strcmp(ControllerMode,'GREEDY')
                [rebPaths, paxPaths,MPCLogs,paxChargeLevels] = GreedyPAMoDRebalancer(station,car,customer,LinkNumVehicles,LinkTime,RebOptions,RoadNetwork,PowerNetwork);
            elseif  strcmp(ControllerMode,'PAMoD')
                [rebPaths, paxPaths,MPCLogs,paxChargeLevels] = PAMoDRebalancer(station,car,customer,LinkNumVehicles,LinkTime,RebOptions,RoadNetwork,PowerNetwork);
            else
                assert(0,'Controller mode not recognized!')
            end
            firstRebalancerCall=0;
            %[rebPaths, paxPaths] = dummy_MPC(vown, outstandingPax, expectedFlows,RoadGraph,RoadCap,LinkTime, StationNodeID);
            vownlog{t}=MPCLogs.vownlog;
            outstandingPaxlog{t}=MPCLogs.outstandingPaxlog;
            custUnassigned=MPCLogs.custUnassigned;
            MPCSolveTimeLog(t)=MPCLogs.solveTime;
        else
            rebCount = rebCount + 1;
        end
        
        %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % assign rebalancing vehicles to routes
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % give the rebPaths to the vehicles
        %fprintf('\n\nWARNING: rebalancing is congestion-unaware. Fix that!\n')
        for i=1:numStations   % Go through stations
            carPtr=0; %We will use this to remove cars from the idle list
            removeFromIdle=[];
            for vv=station(i).carIdle % For each station, go through idle vehicles
                carPtr=carPtr+1;
                carCharge=ceil((car(vv).charge-RebOptions.chargeBias)/RebOptions.chargeProportion); % compute discretized charge level (use same data given to controller)
                if carCharge<1
                    fprintf('Idle vehicle %d has zero charge! Continuing...\n',vv)
                    continue
                end
                try
                    idle_flag=0; % Sometimes the optimizer may just tell us to stay put
                    if (~isempty(rebPaths{StationNodeID(car(vv).ostation),carCharge})) % look up reb routes
                        tmpRebPaths = rebPaths{StationNodeID(car(vv).ostation),carCharge};
                        tmpRebPath=tmpRebPaths{1};
                        if tmpRebPath(2) == -N-StationNodeID(car(vv).ostation) %charging
                            assert(~isempty(find(StationNodeID(car(vv).ostation)==ChargersList,1)),'ERROR: attempting to charge at a non-charger node')
                            assert(length(tmpRebPath)==2,'ERROR: charging path has length >2')
                            car(vv).state = CHARGING;
                            car(vv).dstation = car(vv).ostation;
                            car(vv).dpos=car(vv).pos;
                            car(vv).path = [StationNodeID(car(vv).ostation)];
                            
                            tmpChargerSpeed = RoadNetwork.ChargerSpeed(find(StationNodeID(car(vv).ostation)==ChargersList,1))*RebOptions.chargeProportion;
                            tmpChargerTime  = RoadNetwork.ChargerTime(find(StationNodeID(car(vv).ostation)==ChargersList,1))*RebOptions.predictionTimeStep;
                            car(vv).desired_charge=min(car(vv).charge+tmpChargerSpeed/tmpChargerTime*(Tmincharge*dt),MaxCharge);
                            
                        elseif tmpRebPath(2) == -StationNodeID(car(vv).ostation) %discharging
                            assert(~isempty(find(StationNodeID(car(vv).ostation)==ChargersList,1)),'ERROR: attempting to discharge at a non-charger node')
                            assert(length(tmpRebPath)==2,'ERROR: discharging path has length >2')
                            car(vv).state = DISCHARGING;
                            car(vv).dstation = car(vv).ostation;
                            assert(car(vv).pos(1)==NodesLocation(StationNodeID(car(vv).ostation),1) & car(vv).pos(2)==NodesLocation(StationNodeID(car(vv).ostation),2),'ERROR: idle car is not at a station, why?')
                            car(vv).dpos=car(vv).pos;
                            car(vv).path = [StationNodeID(car(vv).ostation)];
                            
                            tmpChargerSpeed = RoadNetwork.ChargerSpeed(find(StationNodeID(car(vv).ostation)==ChargersList,1))*RebOptions.chargeProportion;
                            tmpChargerTime  = RoadNetwork.ChargerTime(find(StationNodeID(car(vv).ostation)==ChargersList,1))*RebOptions.predictionTimeStep;
                            car(vv).desired_charge=max(car(vv).charge-tmpChargerSpeed/tmpChargerTime*(Tmincharge*dt),1);
                            
                        elseif tmpRebPath(2)>0 && tmpRebPath(2)~=StationNodeID(car(vv).ostation) %the second condition checks that we don't just want to stay where we are
                            car(vv).state = REBALANCING;
                            assert(StationNodeID(car(vv).ostation)==tmpRebPath(1),'ERROR: reb path does not start where it should')
                            car(vv).path = tmpRebPath;
                            car(vv).dpos = NodesLocation(tmpRebPath(end),:);
                            car(vv).dstation = find(StationNodeID == car(vv).path(end),1);
                            
                            LinkNumVehicles(car(vv).path(1), car(vv).path(2)) = LinkNumVehicles(car(vv).path(1), car(vv).path(2))+1;
                            station(car(vv).dstation).carOnRoad = [station(car(vv).dstation).carOnRoad vv];
                        elseif tmpRebPath(2)==StationNodeID(car(vv).ostation)
                            % Do nothing
                            idle_flag=1;
                        else
                            assert(-1,'ERROR: trying to discharge to a different station?')
                        end
                        
                        % update idle cars
                        if ~idle_flag
                            removeFromIdle=[removeFromIdle,vv]; %Will remove these vehicles later
                        end
                        % update rebalancing rebPaths
                        rebPaths{StationNodeID(car(vv).ostation),carCharge}=rebPaths{StationNodeID(car(vv).ostation),carCharge}(2:end);
                    end
                catch
                    aa=2 %Just a place for the debugger
                    disp('Uh-oh: issues with giving reb paths to vehicles')
                    keyboard
                end
            end
            assert(length(station(i).carIdle)==carPtr,'Trivial check for peace of mind (we iterate on a list and count the elements...)')
            for vvv=removeFromIdle %Now we remove the vehicles we assigned earlier
                station(i).carIdle=station(i).carIdle(station(i).carIdle~=vvv);
            end
        end
        
        
        
        % if not empty, interpret route as charge-discharge-rebalance
        %disp('REBALANCING LOOKS WRONG. Look for this message and fix below.')
        %     for i = 1:length(rebPaths)
        %         %disp('TODO: now some of the paths may be empty! Prune them, maybe?')
        %         if isempty(rebPaths{i})
        %             continue
        %         end
        %
        %         stationId = i;
        %
        %         numRebalance = min(length(station(stationId).carIdle), length(rebPaths{i}));
        %
        %
        %         % assign cars to rebalance paths
        %         for rebIndx = 1:numRebalance
        %             tmpCar = station(stationId).carIdle(rebIndx);
        %             car(tmpCar).state = REBALANCING;
        %             assert(StationNodeID(i)==rebPaths{i}{rebIndx}(1,1),'ERROR: reb path does not start where it should')
        %             car(tmpCar).path = rebPaths{i}{rebIndx}(:,1)';
        %             car(tmpCar).dpos = NodesLocation(rebPaths{i}{rebIndx}(end,1),:);
        %             car(tmpCar).dstation = find(StationNodeID == car(tmpCar).path(end),1);
        %             LinkNumVehicles(car(tmpCar).path(1), car(tmpCar).path(2)) = LinkNumVehicles(car(tmpCar).path(1), car(tmpCar).path(2))+1;
        %             station(car(tmpCar).dstation).carOnRoad = [station(car(tmpCar).dstation).carOnRoad tmpCar];
        %         end
        %
        %         % update idle cars
        %         station(stationId).carIdle = station(stationId).carIdle(numRebalance+1:end);
        %
        %         % update rebalancing rebPaths
        %         rebPaths{i} = rebPaths{i}(numRebalance+1:end);
        %
        %     end
        
        
        
        %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % move the vehicles
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        ChargersLoad = zeros(NumChargers,1);
        for i = 1:v
            if car(i).charge<=0
                fprintf('Car %d is at zero charge!\n',i)
                if car(i).state ~= DRIVING_TO_DEST %If we do not have a customer onboard
                    % Find the nearest charger node
                    chargeToNearestCharger=Inf;
                    nearestCharger=-1;
                    pathToNearestCharger=[car(i).path(1)];
                    if length(car(i).path)==1
                        startNode=car(i).path(1);
                    else
                        startNode=car(i).path(2);
                    end
                    for chg=RoadNetwork.ChargersList
                        % path to nearest charger
                        pathToChg = findRoute(startNode, chg, LinkTime,RoadGraph,NodesLocation,findRouteMode,RoadNetwork.Routes);
                        chgToCharger=0;
                        for chgpath=1:length(pathToChg)-1;
                            chgToCharger=chgToCharger+RoadNetwork.ChargeToTraverse(pathToChg(chgpath),pathToChg(chgpath+1));
                        end
                        if chgToCharger<chargeToNearestCharger
                            chargeToNearestCharger=chgToCharger;
                            nearestCharger=chg;
                            pathToNearestCharger=pathToChg;
                        end
                    end
                    % Sanity check - in the current implementation we should
                    % ALWAYS go to the charger where we are
                    if nearestCharger~=startNode
                        fprintf('Weird: car %d is at zero charge at node %d, but it wants to go to node %d to charge instead. Look into it!\n',i,startNode,nearestCharger)
                        keyboard
                    end
                    % Limp home to the node
                    %Remove from carIdle or carOnRoad, drop assigned
                    %customer
                    if car(i).state == IDLE
                        station(car(i).ostation).carIdle=station(car(i).ostation).carIdle(station(car(i).ostation).carIdle~=car(i).id);
                    end
                    if car(i).state == REBALANCING || car(i).state == DRIVING_TO_STATION
                        station(car(i).dstation).carOnRoad = station(car(i).dstation).carOnRoad(station(car(i).dstation).carOnRoad~=car(i).id);
                    end
                    if car(i).state == DRIVING_TO_PICKUP
                        %Drop the customer you were carrying
                        station(customer(car(i).passId).ostation).custUnassigned = [car(i).passId, station(customer(car(i).passId).ostation).custUnassigned];
                        car(i).passId=[];
                    end
                    
                    car(i).state = LIMP_HOME;
                    if length(car(i).path)==1
                        
                        car(i).path = pathToNearestCharger;
                    else
                        % Limp to nearest charger
                        car(i).path = [car(i).path(1), pathToNearestCharger];
                    end
                    car(i).dpos = NodesLocation(car(i).path(end),:);
                    % Charge
                    if length(car(i).path)==1
                        % just start charging
                        car(i).state = CHARGING;
                        car(i).dstation = car(i).ostation;
                        car(i).dpos=car(i).pos;
                        car(i).desired_charge = 1;
                    end
                else
                    fprintf('Car %d with customer on board has negative charge. This case is not handled - we just keep driving and hope we will not drop below the drop dead charge.\n',i)
                end
            end
            if car(i).charge<settings.drop_dead_charge
                if ~find(DropDeadCars==i,1)
                    DropDeadCars = [DropDeadCars, i];
                    fprintf('Car %d has insufficient charge to continue (charge %f). It will not move further!\nLook into it: here is the keyboard...\n',i,car(i).charge);
                    keyboard
                    continue
                else
                    fprintf('Car %d still has insufficient charge to continue (charge %f).\n',i,car(i).charge);
                end
            end
            if car(i).charge>=settings.drop_dead_charge && length(car(i).path) > 1 && car(i).state ~= CHARGING && car(i).state~=DISCHARGING
                if norm(NodesLocation(car(i).path(2),:)-car(i).pos) < LinkSpeed(car(i).path(1), car(i).path(2))*dt
                    % need to go to the next node (or stop)
                    dt1 = norm(NodesLocation(car(i).path(2),:)-car(i).pos) / LinkSpeed(car(i).path(1), car(i).path(2));
                    dt2 = dt - dt1;
                    % We assume that (i) charge required to traverse a link is
                    % ChargeToTraverse and (ii) discharge is linear in time
                    car(i).charge = car(i).charge - ChargeToTraverse(car(i).path(1), car(i).path(2))*dt1/LinkTime(car(i).path(1), car(i).path(2));
                    
                    if length(car(i).path) > 2
                        LinkNumVehicles(car(i).path(1), car(i).path(2)) = LinkNumVehicles(car(i).path(1), car(i).path(2)) - 1;
                        car(i).path = car(i).path(2:end);
                        residualDistance = LinkSpeed(car(i).path(1), car(i).path(2))*dt2;
                        car(i).direction = (NodesLocation(car(i).path(2),:) - NodesLocation(car(i).path(1),:));
                        car(i).direction = car(i).direction./norm(car(i).direction);
                        car(i).pos = NodesLocation(car(i).path(1),:) + car(i).direction*residualDistance;
                        LinkNumVehicles(car(i).path(1), car(i).path(2)) = LinkNumVehicles(car(i).path(1), car(i).path(2)) + 1;
                        %Charge on second link. Note that now path(1) and
                        %path(2) are what used to be path(2),path(3).
                        car(i).charge = car(i).charge - ChargeToTraverse(car(i).path(1), car(i).path(2))*dt2/LinkTime(car(i).path(1), car(i).path(2));
                    else
                        car(i).pos = NodesLocation(car(i).path(2),:);
                        car(i).charge = car(i).charge - ChargeToTraverse(car(i).path(1), car(i).path(2))*dt1/LinkTime(car(i).path(1), car(i).path(2));
                    end
                    
                else
                    % don't need to go to the next node
                    car(i).direction = (NodesLocation(car(i).path(2),:) - NodesLocation(car(i).path(1),:));
                    car(i).direction = car(i).direction./norm(car(i).direction);
                    car(i).pos = car(i).pos + car(i).direction*LinkSpeed(car(i).path(1), car(i).path(2))*dt;
                    car(i).charge = car(i).charge - ChargeToTraverse(car(i).path(1), car(i).path(2))*dt/LinkTime(car(i).path(1), car(i).path(2));
                end
                
            end
            
            if car(i).state == CHARGING
                assert(length(car(i).path)==1 && car(i).pos(1) == NodesLocation(car(i).path(1),1) && car(i).pos(2) == NodesLocation(car(i).path(1),2) && ~isempty(find(ChargersList==car(i).path(1),1)),'Vehicle %d wants to charge but it is not at a charger (at node %d)\n',i,car(i).pos)
                
                tmpcharger=find(ChargersList==car(i).path(1));
                car(i).charge = car(i).charge + ChargerSpeedSim(tmpcharger)*dt;
                ChargersLoad(tmpcharger)=ChargersLoad(tmpcharger)+RoadNetwork.ChargerSpeed(tmpcharger)*RoadNetwork.ChargeUnitToPowerUnit; %This is the increase in charge per SECOND.
            end
            if car(i).state == DISCHARGING
                assert(length(car(i).path)==1 && car(i).pos(1) == NodesLocation(car(i).path(1),1) && car(i).pos(2) == NodesLocation(car(i).path(1),2) && find(ChargersList==car(i).path(1)),'Vehicle %d wants to discharge but it is not at a charger (at node %d)\n',i,car(i).path(1))
                tmpcharger=find(ChargersList==car(i).path(1));
                car(i).charge = car(i).charge - ChargerSpeedSim(tmpcharger)*dt;
                ChargersLoad(tmpcharger)=ChargersLoad(tmpcharger)-PowerNetwork.v2g_efficiency*RoadNetwork.ChargerSpeed(tmpcharger)*RoadNetwork.ChargeUnitToPowerUnit;
            end
        end
        %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Simulate power network with current load
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        %         PNThor=1;
        %         mytime = max(round(t*dt/settings.predictionTimeStep),1);
        %
        %         PNRoadNetwork=RoadNetwork;
        %         PNRoadNetwork.TVRoadCap=zeros(PNThor,RoadNetwork.N,RoadNetwork.N);
        %         PNRoadNetwork.TVRoadCap(PNThor,:,:)=PNRoadNetwork.RoadCap;
        %         PNRoadNetwork.TravelTimes=max(round(LinkTime/settings.predictionTimeStep),LinkTime>0);
        %         PNRoadNetwork.TravelDistance=RoadNetwork.LinkLength/1e3;
        %
        %         PNPowerNetwork=PowerNetwork;
        %         PNPowerNetwork.PowerGensMax=PowerNetwork.PowerGensMax(mytime,:);
        %         PNPowerNetwork.PowerGensMin=PowerNetwork.PowerGensMin(mytime,:);
        %         if t==1
        %             PreviousPowerGens=zeros(1,length(PNPowerNetwork.PowerGensList));
        %         else
        %             PNPowerNetwork.PowerGensMax=min(PowerNetwork.PowerGensMax(mytime,:),PreviousPowerGens+PowerNetwork.PowerRampUp(mytime,:)*dt/settings.predictionTimeStep);
        %             PNPowerNetwork.PowerGensMin=max(PowerNetwork.PowerGensMin(mytime,:),PreviousPowerGens-PowerNetwork.PowerRampDown(mytime,:)*dt/settings.predictionTimeStep);
        %         end
        %         PNPowerNetwork.PowerCosts=PowerNetwork.PowerCosts(mytime,:);
        %         PNPowerNetwork.PowerExtLoads=PowerNetwork.PowerExtLoads(:,mytime);
        %
        %         for g=1:length(ChargersList)
        %             PNPowerNetwork.PowerExtLoads(PowerNetwork.RoadToPowerMap(g))=PNPowerNetwork.PowerExtLoads(PowerNetwork.RoadToPowerMap(g))+ChargersLoad(g);
        %         end
        %
        %         %Ramp-up is handled elsewhere above
        %         PNPowerNetwork.PowerRampUp=Inf*ones(PNThor,length(PowerNetwork.PowerGensList));
        %         PNPowerNetwork.PowerRampDown=Inf*ones(PNThor,length(PowerNetwork.PowerGensList));
        %
        %         PNInitialConditions.EmptyVehicleInitialPosRT=zeros(PNThor,PNRoadNetwork.N,PNRoadNetwork.C);
        %         PNInitialConditions.MinNumVehiclesAti=zeros(length(PNRoadNetwork.RoadGraph),1);
        %
        %         PNRebWeight=0;
        %
        %         %Just to have non-zero sink length
        %         PNPassengers.StarterSinks=[1];
        %         PNPassengers.StarterSources={[1]};
        %         PNPassengers.StarterFlows={[0]};
        %         PNPassengers.Sinks=[1];
        %         PNPassengers.Sources={[1]};
        %         PNPassengers.StartTimes={[PNThor]};
        %         PNPassengers.Flows={[0]};
        %
        %         PNFlags.milpflag=0;
        %         PNFlags.congrelaxflag=1;
        %         PNFlags.sourcerelaxflag=1;
        %         PNFlags.cachedAeqflag=0;%1-RebOptions.firstRebalancerCall;
        %         PNFlags.debugflag=0;
        %         PNFlags.minendchargerelaxflag=1;
        %         PNFlags.endreblocrelaxflag=0;
        %
        %         [cplex_out,fval,exitflag,output,lambdas,dual_prices_ix,dual_charger_prices_ix,lb]=TVPowerBalancedFlow_realtime(PNThor,PNRoadNetwork,PNPowerNetwork,PNInitialConditions,PNRebWeight,PNPassengers,PNFlags);
        %         try
        %             assert(exitflag>-2,'ERROR: power network is infeasible!')
        %         catch
        %             fprintf('\n\nTROUBLE with the power network solution (exit flag %d)!! Keyboard...\n\n',exitflag)
        %             keyboard
        %         end
        %         % Save electricity prices and generator loads across TX
        %         [GenLoads,PowerPrices,ChargerPrices] = TVPowerBalancedFlow_getpowerdata(cplex_out,lambdas,dual_prices_ix,dual_charger_prices_ix,PNThor,PNRoadNetwork,PNPowerNetwork,PNInitialConditions,PNRebWeight,PNPassengers,PNFlags);
        if t==1
            PreviousPowerGens=zeros(1,length(PowerNetwork.PowerGensList));
        end
        [GenLoads,PowerPrices,ChargerPrices,PowerNetworkViolation,DroppedPowerUP,DroppedPowerDN] = TVPowerBalancedFlow_PowerNetworkPrices(t,settings,RoadNetwork,LinkTime,PowerNetwork,PreviousPowerGens,ChargersList,ChargersLoad);
        
        PowerNetworkViolationBool(t) = PowerNetworkViolation;
        PowerNetworkViolationQUP{t} = DroppedPowerUP;
        PowerNetworkViolationQDN{t} = DroppedPowerDN;
        PreviousPowerGens=GenLoads';
        GenLoadsLog(:,t)=GenLoads;
        PowerPricesLog(:,t)=PowerPrices;
        ChargerPricesLog(:,t)=ChargerPrices;
        ChargersLoadLog(:,t)=ChargersLoad;
        
        PowerNetwork.PreviousChargerPrices=ChargerPrices;
        %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % update customer wait and service times, stats
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        for i = 1:cc-1
            if customer(i).pickedup == 0
                customer(i).waitTime = customer(i).waitTime + dt;
            elseif customer(i).pickedup == 1 && customer(i).delivered == 0
                customer(i).serviceTime = customer(i).serviceTime + dt;
            end
        end
        
        % collect customer travel time data
        for i = 1:numStations
            for j = 1:length(station(i).custId)
                if customer(station(i).custId(j)).pickedup == 0
                    avgWaitTimes(t) = avgWaitTimes(t) + customer(station(i).custId(j)).waitTime;
                    cumNumCustomers(t) = cumNumCustomers(t) + 1;
                end
            end
        end
        
        % collect number of vehicles doing stuff
        tot_speed = 0;
        tot_moving_cars = 0;
        not_moving = 0;
        not_a_road = 0;
        for i = 1:v
            if car(i).state ~= IDLE && car(i).state ~= CHARGING && car(i).state ~= DISCHARGING && length(car(i).path)>=2
                tot_speed = tot_speed + LinkSpeed(car(i).path(1), car(i).path(2));
                tot_moving_cars = tot_moving_cars + 1;
                
                if LinkSpeed(car(i).path(1), car(i).path(2)) == 0
                    not_moving = not_moving + 1;
                end
                if ~any(RoadGraph{car(i).path(1)} == car(i).path(2))
                    not_a_road = not_a_road + 1;
                    car(i).state
                end
                numVehiclesBusy(t) = numVehiclesBusy(t) + 1;
                if car(i).state == REBALANCING
                    numVehiclesRebalancing(t) = numVehiclesRebalancing(t) + 1;
                end
                if car(i).state == DRIVING_TO_DEST
                    numVehiclesDtDest(t) = numVehiclesDtDest(t) + 1;
                end
                if car(i).state == DRIVING_TO_PICKUP
                    numVehiclesDtPick(t) = numVehiclesDtPick(t) + 1;
                end
                if car(i).state == DRIVING_TO_STATION
                    numVehiclesDtStat(t) = numVehiclesDtStat(t) + 1;
                end
            end
        end
        if not_moving~=0
            fprintf('%d vehicles are NOT moving.\n',not_moving);
            pause
        end
        if not_a_road~=0
            fprintf('%d vehicles are NOT in a valid road.\n',not_a_road);
            pause
        end
        avgVehicleSpeeds(t) = tot_speed / tot_moving_cars;
        
        % collect "state of unbalance"
        for i = 1:numStations
            idleVehicles(i,t) = length(station(i).carIdle);
            remainingRebPaths(i,t) = length(rebPaths{i});
        end
        desiredDistr = (sum(idleVehicles(:,t)) / numStations) * ones(numStations,1);
        stateOfUnbalance(t) = sum((idleVehicles(:,t) - desiredDistr) .^ 2) / numStations;
        
        % collect number of waiting customers
        waitingCustomerslog{t}=zeros(numStations,1);
        for i=1:numStations
            waitingCustomerslog{t}(i)=length(station(i).custUnassigned);
        end
        
        % collect number of vehicles on each link
        numCarsOnLink{t} = LinkNumVehicles;
        
        % collect vehicle state and location of moving cars
        for i = 1:v
            vehicleStateLog(t,car(i).state+1)=vehicleStateLog(t,car(i).state+1)+1;
            if settings.VideoLog
                if car(i).state == IDLE || car(i).state == CHARGING || car(i).state == DISCHARGING
                    NumIdleCars(t,car(i).path(1)) = NumIdleCars(t,car(i).path(1))+1;
                    IdleCarCharge(t,car(i).path(1)) = IdleCarCharge(t,car(i).path(1))+car(i).charge;
                    if car(i).state == CHARGING
                        NumChargingCars(t,car(i).path(1)) = NumChargingCars(t,car(i).path(1))+1;
                    elseif car(i).state == DISCHARGING
                        NumDischargingCars(t,car(i).path(1)) = NumDischargingCars(t,car(i).path(1))+1;
                    end
                else
                    %MovingCarsLink(t,i,:)=car(i).path(1:2);
                    MovingCarsCharge(t,car(i).path(1),car(i).path(2)) = MovingCarsCharge(t,car(i).path(1),car(i).path(2))+car(i).charge;
                    NumMovingCars2(t,car(i).path(1),car(i).path(2)) = NumMovingCars2(t,car(i).path(1),car(i).path(2)) + 1;
                end
            end
        end
        if settings.VideoLog
            NumMovingCars(t,:,:)=LinkNumVehicles;
            assert(~sum(sum(NumMovingCars(t,:,:)~=NumMovingCars2(t,:,:))),'ERROR: cars are counted incorrectly')
            assert(~sum(sum(xor(NumMovingCars(t,:,:)~=0,MovingCarsCharge(t,:,:)~=0))),'ERROR: cars are counted incorrectly')
            for i=1:N
                IdleCarCharge(t,i)=IdleCarCharge(t,i)/NumIdleCars(t,i);
                for j=1:N
                    MovingCarsCharge(t,i,j)=MovingCarsCharge(t,i,j)/NumMovingCars(t,i,j);
                end
            end

            parfor i=1:v
                MovingCarsLoc(t,i,:)=car(i).pos;
            end
        end
        
        
    end
    
    %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % elaborate and print statistics
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    numDelivered = 0;
    totalServiceTime = 0;
    totalWaitTime = 0;
    for i = 1:cc-1
        if customer(i).delivered
            numDelivered = numDelivered + 1;
            totalServiceTime = totalServiceTime + customer(i).serviceTime;
            totalWaitTime = totalWaitTime + customer(i).waitTime;
        end
    end
    meanServiceTime = totalServiceTime/numDelivered;
    
    allWaitTimes = zeros(numDelivered, 1);
    allServiceTimes = zeros(numDelivered, 1);
    ccc = 1;
    for i = 1:cc-1
        if customer(i).delivered
            allWaitTimes(ccc) = customer(i).waitTime;
            allServiceTimes(ccc) = customer(i).serviceTime;
            ccc = ccc+1;
        end
    end
    
    Stats.meanServiceTime = meanServiceTime;
    Stats.allWaitTimes =allWaitTimes;
    Stats.allServiceTimes=allServiceTimes;
    Stats.rebHist = rebHist;
    Stats.cumNumCustomers = cumNumCustomers;
    Stats.numVehiclesBusy = numVehiclesBusy;
    Stats.numVehiclesRebalancing = numVehiclesRebalancing;
    Stats.numVehiclesNotRebalancing = numVehiclesNotRebalancing;
    Stats.numVehiclesDtDest = numVehiclesDtDest;
    Stats.numVehiclesDtPick = numVehiclesDtPick;
    Stats.numVehiclesDtStat = numVehiclesDtStat;
    Stats.numCarsOnLink = numCarsOnLink;
    Stats.numPlannedRoutes=numPlannedRoutes;
    Stats.numAstarRoutes=numAstarRoutes;
    Stats.numComputedRebalancingRoutes=numComputedRebalancingRoutes;
    Stats.stateOfUnbalance = stateOfUnbalance;
    Stats.remainingRebPaths = remainingRebPaths;
    Stats.idleVehicles = idleVehicles;
    Stats.issuedRebPaths = issuedRebPaths;
    Stats.avgVehicleSpeeds = avgVehicleSpeeds;
    Stats.vownlog = {vownlog{Treb:Treb:Tmax-1}};
    Stats.vownlog_withpax = {vownlog_withpax{Treb:Treb:Tmax-1}};
    Stats.waitingCustomerslog = waitingCustomerslog;
    Stats.outstandingPaxlog = outstandingPaxlog;
    Stats.linkSpeedLog=linkSpeedLog;
    Stats.vehicleStateLog = vehicleStateLog;
    Stats.remainingPaxPaths=remainingPaxPaths;
    Stats.GenLoadsLog=GenLoadsLog;
    Stats.ChargersLoadLog=ChargersLoadLog; 
    Stats.PowerPricesLog=3600/settings.predictionTimeStep*PowerPricesLog; %We convert to prices per hour
    Stats.ChargerPricesLog=3600/settings.predictionTimeStep*ChargerPricesLog;
    Stats.MPCSolveTimeLog=MPCSolveTimeLog;
    Stats.PowerNetworkViolationBool = PowerNetworkViolationBool;
    Stats.PowerNetworkViolationQUP = PowerNetworkViolationQUP;
    Stats.PowerNetworkViolationQDN = PowerNetworkViolationQDN;

    if settings.VideoLog
        Stats.MovingCarsLoc = MovingCarsLoc;
        Stats.MovingCarsCharge = MovingCarsCharge;
        Stats.NumIdleCars = NumIdleCars;
        Stats.NumMovingCars = NumMovingCars;
        Stats.NumMovingCars2= NumMovingCars2;
        Stats.IdleCarCharge = IdleCarCharge;
        Stats.NumChargingCars = NumChargingCars;
        Stats.NumDischargingCars = NumDischargingCars;
    end
    
catch ME
    ME
    keyboard
    disp('Saving...')
    save(['SimulationError_',datestr(now)],'-v7.3')
    rethrow(ME)
end

end

