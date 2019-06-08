% Forked from TVPowerBalancedFlowFinder_sinkbundle

%% Unpack things

C=RoadNetwork.C;
RoadGraph=RoadNetwork.RoadGraph;
TVRoadCap=RoadNetwork.TVRoadCap;
TravelTimes=RoadNetwork.TravelTimes;
TravelDistance=RoadNetwork.TravelDistance;
ChargeToTraverse=RoadNetwork.ChargeToTraverse;
ChargersList=RoadNetwork.ChargersList;
ChargerSpeed=RoadNetwork.ChargerSpeed;
ChargerTime=RoadNetwork.ChargerTime;
%ChargerCost=RoadNetwork.ChargerCost;
ChargerCap=RoadNetwork.ChargerCap;
ChargeUnitToMW=RoadNetwork.ChargeUnitToPowerUnit;
MinEndCharge=RoadNetwork.MinEndCharge;
if MinEndCharge<1
    disp('WARNING: setting MinEndCharge=1')
    MinEndCharge=1;
end
ValueOfTime=RoadNetwork.ValueOfTime;
VehicleCostPerm=RoadNetwork.VehicleCostPerKm/1e3; %Note that the travel distance is in m, not km;
RouteTime=RoadNetwork.RouteTime;
RouteCharge=RoadNetwork.RouteCharge;
try
    BatteryDepreciationPerUnitCharge = RoadNetwork.BatteryDepreciationPerUnitCharge;
catch
    BatteryDepreciationPerUnitCharge = 0;
    disp('Setting battery depreciation at 0')
end

%PowerGraph=PowerNetwork.PowerGraph;
%PowerLineCap=PowerNetwork.PowerLineCap;
%PowerLineReactance=PowerNetwork.PowerLineReactance;
PowerGraphM=PowerNetwork.PowerGraphM;
PowerLineCapM=PowerNetwork.PowerLineCapM;
PowerLineReactanceM=PowerNetwork.PowerLineReactanceM;
PowerGensList=PowerNetwork.PowerGensList;
PowerGensMax=PowerNetwork.PowerGensMax;
PowerGensMin=PowerNetwork.PowerGensMin;
PowerCosts=PowerNetwork.PowerCosts;
PowerExtLoads=PowerNetwork.PowerExtLoads;
RoadToPowerMap=PowerNetwork.RoadToPowerMap;
v2g_efficiency=PowerNetwork.v2g_efficiency;
try
    PowerRampUp=PowerNetwork.PowerRampUp;
catch
    
    PowerRampUp=PowerNetwork.PowerGensMax;
    disp('Setting PowerRampUp as default')
end
try
    PowerRampDown=PowerNetwork.PowerRampDown;
catch
    PowerRampDown=PowerNetwork.PowerGensMax;
    disp('Setting PowerRampDown as default')
end

EmptyVehicleInitialPosRT=InitialConditions.EmptyVehicleInitialPosRT;
MinNumVehiclesAti=InitialConditions.MinNumVehiclesAti;
if length(MinNumVehiclesAti)==1 %If we input a single number
    MinNumVehiclesAti=MinNumVehiclesAti*ones(length(RoadGraph),1);
end


Sources=Passengers.Sources;
Sinks=Passengers.Sinks;
Flows=Passengers.Flows;
StartTimes=Passengers.StartTimes;
StarterSinks=Passengers.StarterSinks;
StarterSources=Passengers.StarterSources;
StarterFlows=Passengers.StarterFlows;

milpflag=Flags.milpflag;
congrelaxflag=Flags.congrelaxflag;
sourcerelaxflag=Flags.sourcerelaxflag;
cachedAeqflag=Flags.cachedAeqflag;
try
    debugflag=Flags.debugflag;
catch
    debugflag=1;        %Makes output verbose
end
try
    minendchargerelaxflag=Flags.minendchargerelaxflag;
catch
    minendchargerelaxflag=1;
    disp('Relaxing min end charge constraint')
end
try
    endreblocrelaxflag=Flags.endreblocrelaxflag;
catch
    endreblocrelaxflag=1;
end
try
   powernetworkloadrelaxflag=Flags.powernetworkloadrelaxflag; 
catch
    powernetworkloadrelaxflag=0;
end
try
    cyclicflag = Flags.cyclicflag;
catch
    cyclicflag = 0;
end

%% Utilities
try
    PowerNetworkRelaxCost = PowerNetwork.VoLL;
catch
    PowerNetworkRelaxCost=6*1e3*1e2/4; %This is per 100 MW.    
    disp('Using default value for VoLL')
end


CongestionCost=5*1e3; %The cost of violating the congestion constraint
SourceRelaxCost=1e5;     % The cost of dropping a source or sink altogether
MinEndChargeRelaxCost = 1e4; %The cost of a vehicle coming in at a too-low charge
EndRebLocRelaxCost = 1e4;


%ChargeUnitToMW=1;     % Converts one unit of charge (quantized) to one MW (for power market)

%Clean up road graph.
for i=1:length(RoadGraph)
    RoadGraph{i}=sort(unique(RoadGraph{i}));
end
% Clean up power graph.
% for i=1:length(PowerGraph)
%     PowerGraph{i}=sort(unique(PowerGraph{i}));
% end


%Nodes in ReverseRoadGraph{i} are such that RoadGraph{ReverseRoadGraph{i}} contains
%i
ReverseRoadGraph=cell(size(RoadGraph));
for i=1:length(RoadGraph)
    for j=RoadGraph{i}
        ReverseRoadGraph{j}=[ReverseRoadGraph{j} i];
    end
end
for i=1:length(ReverseRoadGraph)
    ReverseRoadGraph{i}=sort(unique(ReverseRoadGraph{i}));
end

ReversePowerGraph=cell(size(PowerGraphM));
for i=1:length(PowerGraphM)
    if ~isempty(PowerGraphM)
        for j=PowerGraphM{i}
            ReversePowerGraph{j}=[ReversePowerGraph{j} i];
        end
    end
end
% for i=1:length(ReversePowerGraph)
%     ReversePowerGraph{i}=sort(unique(ReversePowerGraph{i}));
% end


N=length(RoadGraph);
M=length(Sinks);
MS=length(StarterSinks);

E=0;
NumRoadEdges=zeros(N,1);
for i=1:N
    NumRoadEdges(i)=length(RoadGraph{i});
    E=E+length(RoadGraph{i});
end
cumRoadNeighbors=cumsum(NumRoadEdges);
cumRoadNeighbors=[0;cumRoadNeighbors(1:end-1)];

RoadNeighborCounter=sparse([],[],[],N,N,E);
TempNeighVec=zeros(N,1);
for i=1:N
    for j=RoadGraph{i}
        TempNeighVec(j)=1;
    end
    NeighCounterLine=cumsum(TempNeighVec);
    for j=RoadGraph{i}
        RoadNeighborCounter(i,j)=NeighCounterLine(j);
    end
    TempNeighVec=zeros(N,1);
end

NP=length(PowerGraphM);
EP=0;
NumPowerEdges=zeros(NP,1);
for i=1:NP
    NumPowerEdges(i)=length(PowerGraphM{i});
    EP=EP+length(PowerGraphM{i});
end
cumPowerNeighbors=cumsum(NumPowerEdges);
assert(EP==cumPowerNeighbors(end),'ERROR: something wrong with dimensions')
cumPowerNeighbors=[0;cumPowerNeighbors(1:end-1)];




NumGenerators = length(PowerGensList);

NumChargers=length(ChargersList);


% Rearranging for sinks
NumSourcesPerSink=zeros(size(Sinks));
for i=1:length(Sinks)
    NumSourcesPerSink(i)=length(Sources{i});
end
CumNumSourcesPerSink=cumsum(NumSourcesPerSink);
TotNumSources=CumNumSourcesPerSink(end);
CumNumSourcesPerSink=[0; CumNumSourcesPerSink(1:end-1)];

% Rearranging for startersinks
NumSourcesPerStarterSink=zeros(length(StarterSinks),1);
for i=1:length(StarterSinks)
    NumSourcesPerStarterSink(i)=length(StarterSources{i});
end
CumNumStarterSourcesPerSink=cumsum(NumSourcesPerStarterSink);
TotNumStarterSources=sum(NumSourcesPerStarterSink);
CumNumStarterSourcesPerSink=[0; CumNumStarterSourcesPerSink(1:end-1)];

MRT=0; %There are no flows for the customers; FindRoadLink = FindPaxLink (t,c,1,i,j)

StateSize=(E*(MRT+1)*(C)+ 2*(MRT+1)*NumChargers*(C))*Thor + C*TotNumSources + ...
    C*M*Thor + C*Thor*TotNumStarterSources+ C*MS*Thor + C*N + Thor*(NumGenerators + NP + EP);

StateSize = double(StateSize); %in case we get these from Python
% Edge flows for every time step and charge level, for every pax and reb
% Charger flows up and down for every charger and charge level, for every
%   pax and reb
% Departure charge level for every pax
% End time and charge for every pax (note that we have TotNumSources departure
%  locations, but M arrival locations).
% Departure time and charge level for every starter flow (TotNumStarters)
% DEparture time and charge for every starter destination (MS)
% End location and charge of empty vehicles
% Power: Generation level for each time step, phase angle for each node,
%  edge flow

if sourcerelaxflag
    StateSize=StateSize+TotNumSources+TotNumStarterSources;
end
if congrelaxflag
    StateSize=StateSize+E*Thor;
end
if minendchargerelaxflag
    StateSize=StateSize+(MinEndCharge-1)*N;
end
if endreblocrelaxflag
    StateSize=StateSize+N;
end
if powernetworkloadrelaxflag
    StateSize=StateSize+2*NP*Thor;
end
% Passengers on a link and rebalancers on a link at a charge level,
% passengers and rebalancers on a charging/discharging link, passenger
% sources/sinks  at a given charge level, generators, phase angle at each
% power node, power flow on each power link, congestion relaxation,
% source-sink relaxation
if debugflag
    fprintf('State size: %d\n',StateSize)
end


FindRoadLinkPtckij=     @(t,c,k,i,j) (t-1)*(E*(MRT+1)*(C)+ 2*(MRT+1)*NumChargers*(C)) + (c-1)*E*(MRT+1) + (k-1)*E + (cumRoadNeighbors(i) + RoadNeighborCounter(i,j));
FindRoadLinkRtcij=      @(t,c,i,j)   FindRoadLinkPtckij(t,c,MRT+1,i,j);
FindChargeLinkPtckl=    @(t,c,k,i)   (t-1)*(E*(MRT+1)*(C)+ 2*(MRT+1)*NumChargers*(C)) + E*(MRT+1)*(C) + NumChargers*(MRT+1)*(c-1)+NumChargers*(k-1)+i;  %Here we index the charger directly (as opposed to the node hosting the charger)
FindChargeLinkRtcl=     @(t,c,i)     FindChargeLinkPtckl(t,c,MRT+1,i);
FindDischargeLinkPtckl= @(t,c,k,i)   (t-1)*(E*(MRT+1)*(C)+ 2*(MRT+1)*NumChargers*(C)) + E*(MRT+1)*(C) + (MRT+1)*NumChargers*C + NumChargers*(MRT+1)*(c-1)+NumChargers*(k-1)+i;
FindDischargeLinkRtcl=  @(t,c,i)     FindDischargeLinkPtckl(t,c,MRT+1,i);
%FindPaxSourceChargeck=  @(c,k)       (E*(M+1)*(C) + 2*(M+1)*NumChargers*C)*Thor + M*(c-1)+k;
%FindPaxSinkChargetcks  = @(t,c,k,s)  (E*(M+1)*(C) + 2*(M+1)*NumChargers*C)*Thor + C*M + C*TotNumSources*(t-1)+TotNumSources*(c-1)+CumNumSourcesPerSink(k)+s;
FindPaxSourceChargecks=  @(c,k,s)    (E*(MRT+1)*(C) + 2*(MRT+1)*NumChargers*C)*Thor + TotNumSources*(c-1)+CumNumSourcesPerSink(k)+s;
FindPaxSinkChargetck  = @(t,c,k)     (E*(MRT+1)*(C) + 2*(MRT+1)*NumChargers*C)*Thor + C*TotNumSources + C*M*(t-1) + M*(c-1) + k;

FindStarterSourceChargetcks = @(t,c,k,s) (E*(MRT+1)*(C) + 2*(MRT+1)*NumChargers*C)*Thor + C*TotNumSources + Thor*M*C + C*TotNumStarterSources*(t-1)+TotNumStarterSources*(c-1)+CumNumStarterSourcesPerSink(k)+s;
FindStarterSinkChargetck = @(t,c,k)  (E*(MRT+1)*(C) + 2*(MRT+1)*NumChargers*C)*Thor + C*TotNumSources + Thor*M*C + C*TotNumStarterSources*Thor + MS*C*(t-1) + MS*(c-1)+k;

FindEndRebLocationci =  @(c,i)       (E*(MRT+1)*(C) + 2*(MRT+1)*NumChargers*C)*Thor + C*TotNumSources + Thor*M*C + C*TotNumStarterSources*Thor + MS*C*Thor + N*(c-1)+i;

FindGeneratorti =       @ (t,i)      (E*(MRT+1)*(C) + 2*(MRT+1)*NumChargers*C)*Thor + C*TotNumSources + Thor*M*C + C*TotNumStarterSources*Thor + MS*C*Thor + C*N + (NumGenerators + NP + EP)*(t-1) + i; %Note that generators are referenced by their order, not their location in the graph. Similar to sources and sinks.
FindPhaseAngleti =      @ (t,i)      (E*(MRT+1)*(C) + 2*(MRT+1)*NumChargers*C)*Thor + C*TotNumSources + Thor*M*C + C*TotNumStarterSources*Thor + MS*C*Thor + C*N + (NumGenerators + NP + EP)*(t-1) + NumGenerators + i;
%A MAJOR difference in how we identify power links. Now FindPLij(i,j) finds the j-th neighbor of node i (as opposed to link from i to j).
FindPowerLinktij =      @ (t,i,j)    (E*(MRT+1)*(C) + 2*(MRT+1)*NumChargers*C)*Thor + C*TotNumSources + Thor*M*C + C*TotNumStarterSources*Thor + MS*C*Thor + C*N + (NumGenerators + NP + EP)*(t-1) + NumGenerators + NP + cumPowerNeighbors(i) + j;

FindCongRelaxtij =      @ (t,i,j)    (E*(MRT+1)*(C) + 2*(MRT+1)*NumChargers*C)*Thor + C*TotNumSources + Thor*M*C + C*TotNumStarterSources*Thor + MS*C*Thor + C*N + Thor*(NumGenerators + NP + EP) + E*(t-1)+cumRoadNeighbors(i) + RoadNeighborCounter(i,j);

congrelaxoffset=0;
if congrelaxflag
    congrelaxoffset = E*Thor;
end
FindSourceRelaxks =  @ (k,s)         (E*(MRT+1)*(C) + 2*(MRT+1)*NumChargers*C)*Thor + C*TotNumSources + Thor*M*C + C*TotNumStarterSources*Thor + MS*C*Thor + C*N + Thor*(NumGenerators + NP + EP) + congrelaxoffset + CumNumSourcesPerSink(k)+s;
FindStarterSourceRelaxks =@(k,s)     (E*(MRT+1)*(C) + 2*(MRT+1)*NumChargers*C)*Thor + C*TotNumSources + Thor*M*C + C*TotNumStarterSources*Thor + MS*C*Thor + C*N + Thor*(NumGenerators + NP + EP) + congrelaxoffset + TotNumSources + CumNumStarterSourcesPerSink(k)+s;
sourcerelaxoffset=0;
if sourcerelaxflag
    sourcerelaxoffset=TotNumSources+TotNumStarterSources;
end
FindChargeRelaxci =     @(c,i)      (E*(MRT+1)*(C) + 2*(MRT+1)*NumChargers*C)*Thor + C*TotNumSources + Thor*M*C + C*TotNumStarterSources*Thor + MS*C*Thor + C*N + Thor*(NumGenerators + NP + EP)+congrelaxoffset+sourcerelaxoffset+ N*(c-1)+i;
chargerelaxoffset=0;
if minendchargerelaxflag
    chargerelaxoffset=(MinEndCharge-1)*N;
end
FindEndRebLocRelaxi =     @(i)      (E*(MRT+1)*(C) + 2*(MRT+1)*NumChargers*C)*Thor + C*TotNumSources + Thor*M*C + C*TotNumStarterSources*Thor + MS*C*Thor + C*N + Thor*(NumGenerators + NP + EP)+congrelaxoffset+sourcerelaxoffset+chargerelaxoffset+i;
%FindSinkRelaxk   =  @ (k)      E*(M+1)*(C) + 2*(M+1)*NumChargers*C + NumGenerators + NP + EP + E + M + k;
endreblocoffset=0;
if endreblocrelaxflag
    endreblocoffset=N;
end
FindPowerNetworkRelaxUPti = @(t,i)       (E*(MRT+1)*(C) + 2*(MRT+1)*NumChargers*C)*Thor + C*TotNumSources + Thor*M*C + C*TotNumStarterSources*Thor + MS*C*Thor + C*N + Thor*(NumGenerators + NP + EP)+congrelaxoffset+sourcerelaxoffset+chargerelaxoffset+endreblocoffset+NP*(t-1)+i;

FindPowerNetworkRelaxDNti = @(t,i)       (E*(MRT+1)*(C) + 2*(MRT+1)*NumChargers*C)*Thor + C*TotNumSources + Thor*M*C + C*TotNumStarterSources*Thor + MS*C*Thor + C*N + Thor*(NumGenerators + NP + EP)+congrelaxoffset+sourcerelaxoffset+chargerelaxoffset+endreblocoffset+ NP*Thor + NP*(t-1)+i;


%%
try
    DIAGNOSTIC_FLAG;
catch
    DIAGNOSTIC_FLAG=0;
end

try
    if DIAGNOSTIC_FLAG
        fprintf('DIAGNOSTICS:\n')
        
        tt=4;
        
        % First and last nonempty road link
        is=1;
        while (isempty(RoadGraph{is}))
            is=is+1;
        end
        it=N;
        while (isempty(RoadGraph{it}))
            it=it-1;
        end
        for c=1:C
            fprintf('Rebalancing flows at charge level %d: from %d to %d\n',c,FindRoadLinkRtcij(tt,c,is,RoadGraph{is}(1)),FindRoadLinkRtcij(tt,c,it,RoadGraph{it}(end)))
        end
        for c=1:C
            fprintf('Rebalancing recharging flow at charge level %d: from %d to %d\n',c, FindChargeLinkRtcl(tt,c,1),FindChargeLinkRtcl(tt,c,NumChargers))
        end
        for c=1:C
            fprintf('Rebalancing discharging flow at charge level %d: from %d to %d\n',c, FindDischargeLinkRtcl(tt,c,1),FindDischargeLinkRtcl(tt,c,NumChargers))
        end
        fprintf('Next block of entries starts at %d\n',FindRoadLinkRtcij(tt+1,1,is,RoadGraph{is}(1)))
        
        for c=1:C
            fprintf('Source flows for all pax at charge level %d: %d to %d\n',c,FindPaxSourceChargecks(c,1,1),FindPaxSourceChargecks(c,M,length(Sources{M})));
        end
        for c=1:C
            fprintf('Sink flows for all pax at charge level %d and time %d: %d to %d\n',c,1,FindPaxSinkChargetck(1,c,1),FindPaxSinkChargetck(1,c,M));
        end
        
        for c=1:C
            fprintf('Sink flows for all pax at charge level %d and time %d: %d to %d\n',c,Thor,FindPaxSinkChargetck(Thor,c,1),FindPaxSinkChargetck(Thor,c,M));
        end
        for c=1:C
            fprintf('Starter source flows for all pax at charge level %d and time %d: %d to %d\n',c,1,FindStarterSourceChargetcks(1,c,1,1),FindStarterSourceChargetcks(1,c,MS,length(StarterSources{M})));
        end
        for c=1:C
            fprintf('Starter source flows for all pax at charge level %d and time %d: %d to %d\n',c,Thor,FindStarterSourceChargetcks(Thor,c,1,1),FindStarterSourceChargetcks(Thor,c,MS,length(StarterSources{M})));
        end
        for c=1:C
            fprintf('Starter sink flows for all pax at charge level %d and time %d: %d to %d\n',c,1,FindStarterSinkChargetck(1,c,1),FindStarterSinkChargetck(1,c,MS));
        end
        for c=1:C
            fprintf('Starter sink flows for all pax at charge level %d and time %d: %d to %d\n',c,Thor,FindStarterSinkChargetck(Thor,c,1),FindStarterSinkChargetck(Thor,c,MS));
        end
        
        fprintf('End location of empty vehicles: %d to %d\n',FindEndRebLocationci(1,1),FindEndRebLocationci(C,N))
        
        
        %
        
        for tt=[1 Thor];
            fprintf('Generator flows at time %d: %d to %d\n',tt,FindGeneratorti(tt,1),FindGeneratorti(tt,NumGenerators))
            
            fprintf('Phase angles flows at time %d for electric nodes: %d to %d\n',tt,FindPhaseAngleti(tt,1),FindPhaseAngleti(tt,NP))
            
            fprintf('Link flows at time %d: %d to %d\n',tt,FindPowerLinktij(tt,1,1),FindPowerLinktij(tt,NP,length(PowerGraphM{NP})))
            
        end
        if congrelaxflag
            fprintf('Cong. relax entries: %d to %d (there should be %d entries)\n',FindCongRelaxtij(1,is,RoadGraph{is}(1)),FindCongRelaxtij(Thor,it,RoadGraph{it}(end)),E*Thor)
        end
        if sourcerelaxflag
            fprintf('Source relax entries: %d to %d (there should be %d entries)\n',FindSourceRelaxks(1,1),FindSourceRelaxks(M,length(Sources{M})),TotNumSources)
            fprintf('Starter relax entries: %d to %d (there should be %d entries)\n',FindStarterSourceRelaxks(1,1),FindStarterSourceRelaxks(MS,length(Sources{MS})),TotNumStarterSources)
        end
        if minendchargerelaxflag
            fprintf('Min. end. charge relax entries: %d to %d (there should be %d entries)\n',FindChargeRelaxci(1,1),FindChargeRelaxci(MinEndCharge-1,N),(MinEndCharge-1)*N )
        end
        if endreblocrelaxflag
            fprintf('Min. end. location relax entries: %d to %d (there should be %d entries)\n',FindEndRebLocRelaxi(1),FindEndRebLocRelaxi(N),N )
        end
        if powernetworkloadrelaxflag
            fprintf('Power network load relax entries: %d to %d (there should be %d entries)\n',FindPowerNetworkRelaxUPti(1,1),FindPowerNetworkRelaxDNti(Thor,NP),2*Thor*NP )
        end
    end
catch ME
    disp('Something crashed in the diagnostics. Check it out (keyboard)...')
    disp(ME)
    %Diagnostics off
    keyboard
end
