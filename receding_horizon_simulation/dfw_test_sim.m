function [] = dfw_test_sim(controllerMode,fileName,timeSettings,controllerSettings)


addpath(genpath('../solvers')); %Hack to get the functions in the root folder. Will remove.

addpath(genpath('utilities'));
addpath(genpath('rebalancers'));

if nargin<1
    controllerMode = 'PAMoD'
    disp('Setting controller to PAMoD')
end
if nargin<2
    disp('Default filename!')
    fileName.dataName = '../atx/dfw_roadgraph_kmeans_tv_Federico_25cl_19h_900step_vNature4.mat'
    fileName.busLocName = '../atx/dfw_buslocations_25cl_windsun_v3'
    fileName.saveNameSuffix = '_v5'
end
if nargin<3
    disp('Default time settings!')
    timeSettings.timeSteps = 1;              % number of time steps per minute
    timeSteps = timeSettings.timeSteps;
    timeSettings.dt = 60/timeSteps;         % length of each time step (in seconds)
    timeSettings.Tmax = 600*timeSteps;      % simulation time, in steps.
    timeSettings.Treb = 2*timeSteps;
    timeSettings.Thor_reb = 2*60*60; % horizon of the MPC optimizer, here 2 hours, in SECONDS
    timeSettings.predictionTimeStep=30*60;  % time step of the MPC optimizer, here 30 minutes, in SECONDS
    timeSettings.Tmincharge = 2*timeSteps; % minimum duration of a charging-discharging action
    timeSettings.Tthresh = 0;     % at what time to start gathering data
end
if nargin<4
    disp('Default controller settings!')
    controllerSettings.v = 1.5*1e5;
    controllerSettings.NAIVEFLAG = 0;
    controllerSettings.UNPLANNEDPAXFLAG = 1; % If set, whenever a precomputed pax route is not available, calls A*. If unset, waits until next rebalancing horizon.
    controllerSettings.forecastPeriods = 10;
    %controllerSettings.initialCarCharge = round(C/2);
    %controllerSettings.MinEndCharge = round(C/2);
    controllerSettings.drop_dead_charge=-2; %The car will stop moving if the charge drops below this level
    controllerSettings.findRouteMode = 2; %1: use A*. 2: use cached routes
    controllerSettings.VideoLog = 1;
    controllerSettings.customerStdDev = 0;
    controllerSettings.powerStdDev = 0;
end

%%

% rUNs the DFW scenario with the new function
addpath(genpath('/home/frossi2/mosek/8/toolbox/r2014a'));
addpath(genpath('/opt/ibm/ILOG/CPLEX_Studio128/cplex/matlab/x86-64_linux'))
addpath(genpath('/home/frossi2/glpkmex/'));
%%
disp('Building road network')
RoadCapMultiplier=1;

%LoadRoadGraphLarge;
addpath('../')
%load('../atx/dfw_roadgraph_kmeans_tv_federico_25cl_windsun_24h_v2.mat')
% load('../atx/dfw_roadgraph_kmeans_tv_federico_25cl_windsun_v2.mat')
%load('../atx/dfw_roadgraph_kmeans_tv_federico_25cl_windsun_19h_v2');
%disp('OVERRIDING NUMBER OF VEHICLES - press any key to acknowledge')
%num_vehicles=250000;
%pause
load(fileName.dataName);
load(fileName.busLocName);

timeStepSize=double(timeStepSize);

RoadNetwork.C = C;
RoadNetwork.RoadGraph = RoadGraph;
RoadNetwork.RoadCap = double(RoadCap)*RoadCapMultiplier;
RoadNetwork.NodesLonLat=NodesLonLat;
RoadNetwork.RefLonLat=RefLonLat;
RoadNetwork.NodesLocation = lla2flat([NodesLonLat(:,2),NodesLonLat(:,1),zeros(size(NodesLonLat(:,1)))], [RefLonLat(2),RefLonLat(1)], 0, 0);%28.8;
RoadNetwork.NodesLocation=[RoadNetwork.NodesLocation(:,2),RoadNetwork.NodesLocation(:,1)];
RoadNetwork.N = length(RoadGraph);

RoadNetwork.LinkLength = zeros(RoadNetwork.N,RoadNetwork.N);
for i=1:RoadNetwork.N
    for j=RoadGraph{i}
        RoadNetwork.LinkLength(i,j)=norm(RoadNetwork.NodesLocation(j,:)-RoadNetwork.NodesLocation(i,:),2);
    end
end
RoadNetwork.LinkLength=sparse(RoadNetwork.LinkLength);

%RoadNetwork.LinkLength = TravelDistance; %We ignore the travel distance
RoadNetwork.LinkFreeFlowSpeed = RoadNetwork.LinkLength./SimTravelTimes;
RoadNetwork.LinkFreeFlowSpeed(isnan(RoadNetwork.LinkFreeFlowSpeed )) = 0;

RoadNetwork.ChargeToTraverse = double(ChargeReqs);
RoadNetwork.ChargersList = double(ChargerList);
RoadNetwork.ChargerSpeed = double(ChargerSpeeds);
RoadNetwork.ChargerTime = double(ChargerTimes);
RoadNetwork.ChargerTimesSim = RoadNetwork.ChargerTime*timeStepSize; %Minimum duration of charge in seconds
RoadNetwork.ChargerSpeedSim = RoadNetwork.ChargerSpeed./RoadNetwork.ChargerTimesSim; %Charge units per second

%RoadNetwork.ChargeUnitToEnergyUnit=ChargeUnitToEnergyUnit;
RoadNetwork.ChargeUnitToPowerUnit=ChargeUnitToPowerUnit;
RoadNetwork.ChargerCap = ChargerCaps;
if nargin<4
    RoadNetwork.MinEndCharge=round(C/2);
else
    RoadNetwork.MinEndCharge = controllerSettings.MinEndCharge;
end
    %fprintf('\n\nWARNING: REDUCED MINENDCHARGE to C/2!\n\n')
RoadNetwork.ValueOfTime=ValueOfTime;
RoadNetwork.VehicleCostPerKm=VehicleCostPerKm;
RoadNetwork.BatteryDepreciationPerUnitCharge = BatteryDepreciationPerUnitCharge;

%LoadStationLocations;
RoadNetwork.StationNodeID = 1:1:RoadNetwork.N;
RoadNetwork.StationLocation = RoadNetwork.NodesLocation;

disp('Building routes')
[RouteTime,RouteCharge,Routes] = build_routes(RoadGraph,TravelTimes,ChargeReqs);
RoadNetwork.Routes=Routes;
RoadNetwork.RouteTime=RouteTime;
RoadNetwork.RouteCharge=RouteCharge;

%%
disp('Building power network')
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
%PowerNetwork.PowerToRoadMap=PowerToRoadMap;
PowerNetwork.v2g_efficiency=v2g_efficiency;
PowerNetwork.PowerRampUp=PowerRampUp;
PowerNetwork.PowerRampDown=PowerRampDown;
try
    PowerNetwork.VoLL = VoLL;
catch
    VoLL_kW = 6;
    PowerNetwork.VoLL = VoLL_kW * 1e3 * BaseMVA * timeStepSize/3600;
    disp('Setting VoLL inside MATLAB')
end
try
    PowerNetwork.VoLL_threshold = VoLL_threshold;
catch
    VoLL_threshold_KWh = 5;
    PowerNetwork.VoLL_threshold = VoLL_threshold_KWh * 1e-3/BaseMVA * timeStepSize/3600;
    disp('Setting VoLL threshols inside MATLAB')
end

%% Simulation parameters
disp('Building sim parameters')
timeSteps = timeSettings.timeSteps;                      % number of time steps per minute
settings.dt = timeSettings.dt;         % length of each time step (in seconds)
settings.Tmax = timeSettings.Tmax;      % simulation time, in steps.
settings.Treb = timeSettings.Treb;        % how often do we rebalance, in steps.

settings.Thor_reb = timeSettings.Thor_reb; % horizon of the MPC optimizer, here 2 hours, in SECONDS
settings.predictionTimeStep=timeSettings.predictionTimeStep;  % time step of the MPC optimizer, here 30 minutes, in SECONDS
settings.Tmincharge = timeSettings.Tmincharge; % minimum duration of a charging-discharging action


settings.Tthresh = timeSettings.Tthresh;     % at what time to start gathering data

settings.v = controllerSettings.v;
settings.NAIVEFLAG = controllerSettings.NAIVEFLAG;
settings.UNPLANNEDPAXFLAG = controllerSettings.UNPLANNEDPAXFLAG; % If set, whenever a precomputed pax route is not available, calls A*. If unset, waits until next rebalancing horizon.


settings.forecastPeriods = controllerSettings.forecastPeriods;
if nargin<4
    settings.initialCarCharge = round(C/2);
else
    settings.initialCarCharge = controllerSettings.initialCarCharge;
end
settings.drop_dead_charge=controllerSettings.drop_dead_charge; %The car will stop moving if the charge drops below this level

settings.findRouteMode = controllerSettings.findRouteMode; %1: use A*. 2: use cached routes

settings.ControllerMode=controllerMode; %GREEDY or PAMoD
settings.VideoLog = controllerSettings.VideoLog;

settings.customerStdDev = controllerSettings.customerStdDev;
settings.powerStdDev = controllerSettings.powerStdDev;

%% Customer data
disp('Building customer data')
% StartTimes: the start time slot (each slot has a length of timeStepSize
% s)
% Sources: the start station
% Sinks: the end station
% FlowsIn: the number of pax

% For every customer
% randomly sample a start time within the slot
% set start location: location of start station
% set end location: location of end station
% We preallocate the customer data vectors generously.
arrivalTimes=zeros(round(sum(FlowsIn)*(1+4*settings.customerStdDev)),1);
departureLocations=zeros(round(sum(FlowsIn)*(1+4*settings.customerStdDev)),2);
arrivalLocations=zeros(round(sum(FlowsIn)*(1+4*settings.customerStdDev)),2);


DepTimeSlots=sort(unique(StartTimes));
paxcount=1;
for depTime=DepTimeSlots
    %numPax(depTime) = sum(FlowsIn(StartTimes==depTime));
    
    for i=1:RoadNetwork.N
        for j=1:RoadNetwork.N
            % Here we add noise to the actual number of pax for each OD pair
            numPaxij=round(sum(FlowsIn(StartTimes==depTime & Sources==i & Sinks==j))*(1+settings.customerStdDev*randn()));
            arrivalTimes(paxcount:paxcount+numPaxij-1)=timeStepSize*(depTime-1) + rand(numPaxij,1)*timeStepSize;
            departureLocations(paxcount:paxcount+numPaxij-1,:)=repmat(RoadNetwork.NodesLocation(RoadNetwork.StationNodeID(i),:),[numPaxij,1]);
            arrivalLocations(paxcount:paxcount+numPaxij-1,:)=repmat(RoadNetwork.NodesLocation(RoadNetwork.StationNodeID(j),:),[numPaxij,1]);
            paxcount=paxcount+numPaxij;
        end
    end  
end
arrivalTimes=arrivalTimes(1:paxcount-1);
departureLocations=departureLocations(1:paxcount-1,:);
arrivalLocations=arrivalLocations(1:paxcount-1,:);

% filename = 'SimTripData1820.csv';
% MData = csvread(filename);
% arrivalTimeOffset = 18*60*timeSteps;
% arrivalTimes = (MData(:,3)*3600 + MData(:,4)*60 + MData(:,5))/60*timeSteps - arrivalTimeOffset;
Trips.arrivalTimes       = arrivalTimes/60*timeSteps;
Trips.departureLocations = departureLocations;
Trips.arrivalLocations   = arrivalLocations;

[Trips.arrivalTimes,sortedTimesIdx]=sort(Trips.arrivalTimes);
Trips.departureLocations=Trips.departureLocations(sortedTimesIdx,:);
Trips.arrivalLocations=Trips.arrivalLocations(sortedTimesIdx,:);

Trips.predictFileName = fileName.dataName;
% %%
% for i=1:RoadNetwork.N
%     for j=RoadGraph{i}
%         if RoadNetwork.LinkLength(i,j)==0
%             fprintf('WARNING: link %d-%d exists but has zero length\n',i,j)
%         end
%     end
% end

%% Go!
disp('Starting simulation')
if strcmp(settings.ControllerMode,'POWER')
    Stats = simulate_network_only_power(RoadNetwork, PowerNetwork, Trips, settings);
else
    Stats = simulate_network_with_power(RoadNetwork, PowerNetwork, Trips, settings);
end
%%

if ~strcmp(settings.ControllerMode,'POWER')
    cumWaitPaxNum=zeros(length(RoadNetwork.StationNodeID),settings.Tmax);
    for t=1:length(Stats.waitingCustomerslog)-1
        cumWaitPaxNum(:,t)=Stats.waitingCustomerslog{t};
    end
    colormap('default')
    surf(cumWaitPaxNum,'EdgeColor','none','LineStyle','none','FaceLighting','phong')
    title('Waiting passengers')
    
    figure()
    surf(Stats.idleVehicles,'EdgeColor','none','LineStyle','none','FaceLighting','phong')
    title('Idle vehicles')
    %
    % figure()
    % surf(Stats.remainingRebPaths,'EdgeColor','none','LineStyle','none','FaceLighting','phong')
    % title('Remaining reb paths')
    
    figure()
    townmat=zeros(length(RoadNetwork.StationNodeID),length(Stats.vownlog));
    for t=1:length(Stats.vownlog)
        townmat(:,t)=Stats.vownlog{t};
    end
    surf(townmat,'EdgeColor','none','LineStyle','none','FaceLighting','phong')
    title('Own vehicles')
    
    figure()
    townpaxmat=zeros(length(RoadNetwork.StationNodeID),length(Stats.vownlog_withpax));
    for t=1:length(Stats.vownlog)
        townpaxmat(:,t)=Stats.vownlog{t};
    end
    surf(townpaxmat,'EdgeColor','none','LineStyle','none','FaceLighting','phong')
    title('Own vehicles (with pax)')
    
    figure()
    avglinkspeed=zeros(settings.Tmax-1,1);
    for t=1:settings.Tmax-1
        E=0;
        for i=1:length(RoadGraph)
            E=E+length(RoadGraph{i});
            if ~isempty(RoadGraph{i})
                for j=RoadGraph{i}
                    avglinkspeed(t)=avglinkspeed(t) + Stats.linkSpeedLog{t}(i,j);
                end
            end
        end
        avglinkspeed(t)=avglinkspeed(t)/E;
    end
    plot(avglinkspeed)
    title('Average link speed')
    
    figure()
    area([Stats.vehicleStateLog(2:2:end,2:end),Stats.vehicleStateLog(2:2:end,1)])
    legend('DRIVING TO PICKUP','DRIVING TO DEST ', 'DRIVING TO STATION ','REBALANCING','CHARGING','DISCHARGING','LIMP HOME','Idle');
    title('Vehicle state')
    
    fprintf('Avg. (real) service time: %f, wait time %f, travel time %f, num. customers delivered: %d/%d\n',mean(Stats.allServiceTimes+Stats.allWaitTimes),mean(Stats.allWaitTimes),mean(Stats.allServiceTimes),length(Stats.allWaitTimes),length(arrivalTimes(arrivalTimes>=0 & arrivalTimes<=settings.Tmax)));
    fprintf('Easy-to-copy times:\n %f \t %f\n',mean(Stats.allServiceTimes+Stats.allWaitTimes),mean(Stats.allWaitTimes))

end

figure()
errorbar(mean(Stats.PowerPricesLog),std(Stats.PowerPricesLog))
hold all
plot(max(Stats.PowerPricesLog),':')
plot(min(Stats.PowerPricesLog),':')
title('Power prices')
xlabel('Time')
ylabel('Price ($/MWh)')


PowerStats = ExtractSimPowerStats(settings,Stats,PowerNetwork,isbusinDallas);

save([fileName.saveFolder,'DFWSim_',settings.ControllerMode,'_10h_',num2str(settings.Thor_reb),'horizon_',datestr(now),fileName.saveNameSuffix],'-v7.3')
%save(['CDC17_iCRRP_RT_nocong_CReP',num2str(settings.CRePFLAG),'_AStar',num2str(settings.ASTARFLAG),'_RoadCap',num2str(RoadCapMultiplier),'_',num2str(settings.Tmax),'_',char(datetime('now')),'.mat'])
