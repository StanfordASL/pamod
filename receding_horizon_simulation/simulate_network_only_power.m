function Stats = simulate_network_only_power(RoadNetwork, PowerNetwork, Trips, settings)

% Syntax:
% Stats = simulate_network_only_power(RoadNetwork, PowerNetwork, Trips, settings)
%

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


%% state definitions
LoadStateDefinitions;

GenLoadsLog = zeros(length(PowerNetwork.PowerGensList),Tmax);
PowerPricesLog = zeros(length(PowerNetwork.PowerGraphM),Tmax);
ChargerPricesLog = zeros(length(RoadNetwork.ChargersList),Tmax);
ChargersLoadLog =  zeros(length(RoadNetwork.ChargersList),Tmax);

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
PowerNetworkViolationBool = zeros(Tmax,1);
PowerNetworkViolationQUP = cell(Tmax,1);
PowerNetworkViolationQDN = cell(Tmax,1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Begin Simulation: main loop
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

try %Wrapped in a try-catch block so that, if anything goes wrong, we get access to the internal state
    
    for t = 1:Tmax-1
        
        DispHour = floor(t*dt/3600);
        DispMin  = floor(t*dt/60)-60*DispHour;
        DispSec  = t*dt-60*DispMin-60*60*DispHour;
        fprintf('POWER: t=%d (%0.2d:%0.2d:%0.2d)\n',...
            t,DispHour,DispMin,DispSec)
       
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
        ChargersLoad = zeros(NumChargers,1);
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
               
    end
    
    %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % elaborate and print statistics
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Stats.GenLoadsLog=GenLoadsLog;
    Stats.ChargersLoadLog=ChargersLoadLog;
    Stats.PowerPricesLog=3600/settings.predictionTimeStep*PowerPricesLog; %We convert to prices per hour
    Stats.ChargerPricesLog=3600/settings.predictionTimeStep*ChargerPricesLog;
    Stats.PowerNetworkViolationBool = PowerNetworkViolationBool;
    Stats.PowerNetworkViolationQUP = PowerNetworkViolationQUP;
    Stats.PowerNetworkViolationQDN = PowerNetworkViolationQDN;
    
catch ME
    ME
    disp('Saving...')
    save(['SimulationError_',datestr(now)],'-v7.3')
    rethrow(ME)
end

end
