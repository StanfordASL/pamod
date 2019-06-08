%% Unpack things

C=RoadNetwork.C;
RoadGraph=RoadNetwork.RoadGraph;
RoadCap=RoadNetwork.RoadCap;
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
ValueOfTime=RoadNetwork.ValueOfTime;
VehicleCostPerm=RoadNetwork.VehicleCostPerKm/1e3; %Note that the travel distance is in m, not km
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


FullVehicleInitialPos =InitialConditions.FullVehicleInitialPos;
EmptyVehicleInitialPos=InitialConditions.EmptyVehicleInitialPos;

Sources=Passengers.Sources;
Sinks=Passengers.Sinks;
Flows=Passengers.Flows;
StartTimes=Passengers.StartTimes;

milpflag=Flags.milpflag;
congrelaxflag=Flags.congrelaxflag;
sourcerelaxflag=Flags.sourcerelaxflag;
cachedAeqflag=Flags.cachedAeqflag;
try
   powernetworkloadrelaxflag=Flags.powernetworkloadrelaxflag; 
catch
    powernetworkloadrelaxflag=0;
end
%% Utilities
try
    PowerNetworkRelaxCost = PowerNetwork.VoLL;
catch
    PowerNetworkRelaxCost=6*1e3*1e2/4;% 1e7; %This is per 100 MW. 
    disp('Using default value for VoLL')
end

debugflag=1;        %Makes output verbose

CongestionCost=5*1e7; %The cost of violating the congestion constraint
SourceRelaxCost=1e9;     % The cost of dropping a source or sink altogether
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


StateSize=(E*(M+1)*(C)+ 2*(M+1)*NumChargers*(C))*Thor + C*TotNumSources + ...
    C*M*Thor + C*N + Thor*(NumGenerators + NP + EP);
StateSize = double(StateSize);
% Edge flows for every time step and charge level, for every pax and reb
% Charger flows up and down for every charger and charge level, for every
%   pax and reb
% Departure charge level for every pax
% End time and charge for every pax (note that we have TotNumSources departure
%  locations, but M arrival locations).
% End location and charge of empty vehicles
% Power: Generation level for each time step, phase angle for each node,
%  edge flow

if sourcerelaxflag
    StateSize=StateSize+TotNumSources;
end
if congrelaxflag
    StateSize=StateSize+E*Thor;
end
if powernetworkloadrelaxflag
    StateSize=StateSize+NP*Thor;
end
% Passengers on a link and rebalancers on a link at a charge level,
% passengers and rebalancers on a charging/discharging link, passenger
% sources/sinks  at a given charge level, generators, phase angle at each
% power node, power flow on each power link, congestion relaxation,
% source-sink relaxation
if debugflag
    fprintf('State size: %d\n',StateSize)
end


FindRoadLinkPtckij=     @(t,c,k,i,j) (t-1)*(E*(M+1)*(C)+ 2*(M+1)*NumChargers*(C)) + (c-1)*E*(M+1) + (k-1)*E + (cumRoadNeighbors(i) + RoadNeighborCounter(i,j));
FindRoadLinkRtcij=      @(t,c,i,j)   FindRoadLinkPtckij(t,c,M+1,i,j);
FindChargeLinkPtckl=    @(t,c,k,i)   (t-1)*(E*(M+1)*(C)+ 2*(M+1)*NumChargers*(C)) + E*(M+1)*(C) + NumChargers*(M+1)*(c-1)+NumChargers*(k-1)+i;  %Here we index the charger directly (as opposed to the node hosting the charger)
FindChargeLinkRtcl=     @(t,c,i)     FindChargeLinkPtckl(t,c,M+1,i);
FindDischargeLinkPtckl= @(t,c,k,i)   (t-1)*(E*(M+1)*(C)+ 2*(M+1)*NumChargers*(C)) + E*(M+1)*(C) + (M+1)*NumChargers*C + NumChargers*(M+1)*(c-1)+NumChargers*(k-1)+i;
FindDischargeLinkRtcl=  @(t,c,i)     FindDischargeLinkPtckl(t,c,M+1,i);

%FindPaxSourceChargeck=  @(c,k)       (E*(M+1)*(C) + 2*(M+1)*NumChargers*C)*Thor + M*(c-1)+k;
%FindPaxSinkChargetcks  = @(t,c,k,s)  (E*(M+1)*(C) + 2*(M+1)*NumChargers*C)*Thor + C*M + C*TotNumSources*(t-1)+TotNumSources*(c-1)+CumNumSourcesPerSink(k)+s;
FindPaxSourceChargecks=  @(c,k,s)       (E*(M+1)*(C) + 2*(M+1)*NumChargers*C)*Thor + TotNumSources*(c-1)+CumNumSourcesPerSink(k)+s;
FindPaxSinkChargetck  = @(t,c,k)  (E*(M+1)*(C) + 2*(M+1)*NumChargers*C)*Thor + C*TotNumSources + C*M*(t-1) + M*(c-1) + k;


FindEndRebLocationci =  @(c,i)       (E*(M+1)*(C) + 2*(M+1)*NumChargers*C)*Thor + C*TotNumSources + Thor*M*C + N*(c-1)+i;

FindGeneratorti =       @ (t,i)      (E*(M+1)*(C) + 2*(M+1)*NumChargers*C)*Thor + C*TotNumSources + Thor*M*C + C*N + (NumGenerators + NP + EP)*(t-1) + i; %Note that generators are referenced by their order, not their location in the graph. Similar to sources and sinks.
FindPhaseAngleti =      @ (t,i)      (E*(M+1)*(C) + 2*(M+1)*NumChargers*C)*Thor + C*TotNumSources + Thor*M*C + C*N + (NumGenerators + NP + EP)*(t-1) + NumGenerators + i;
%A MAJOR difference in how we identify power links. Now FindPLij(i,j) finds the j-th neighbor of node i (as opposed to link from i to j). 
FindPowerLinktij =      @ (t,i,j)    (E*(M+1)*(C) + 2*(M+1)*NumChargers*C)*Thor + C*TotNumSources + Thor*M*C + C*N + (NumGenerators + NP + EP)*(t-1) + NumGenerators + NP + cumPowerNeighbors(i) + j;

FindCongRelaxtij =      @ (t,i,j)    (E*(M+1)*(C) + 2*(M+1)*NumChargers*C)*Thor + C*TotNumSources + Thor*M*C + C*N + Thor*(NumGenerators + NP + EP) + E*(t-1)+cumRoadNeighbors(i) + RoadNeighborCounter(i,j);
congrelaxoffset=0;
if congrelaxflag
    congrelaxoffset = E*Thor;
end
FindSourceRelaxks =  @ (k,s)         (E*(M+1)*(C) + 2*(M+1)*NumChargers*C)*Thor + C*TotNumSources + Thor*M*C + C*N + Thor*(NumGenerators + NP + EP) + congrelaxoffset + CumNumSourcesPerSink(k) + s;
sourcerelaxoffset=0;
if sourcerelaxflag
    sourcerelaxoffset=TotNumSources;
end
FindPowerNetworkRelaxti = @(t,i)     (E*(M+1)*(C) + 2*(M+1)*NumChargers*C)*Thor + C*TotNumSources + Thor*M*C + C*N + Thor*(NumGenerators + NP + EP) + congrelaxoffset + sourcerelaxoffset + NP*(t-1) + i;

%%
try
    DIAGNOSTIC_FLAG;
catch
    DIAGNOSTIC_FLAG=0;
end

if DIAGNOSTIC_FLAG
    fprintf('DIAGNOSTICS:\n')
    
    tt=2;
    
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
        for k=1:M
            fprintf('Passenger %d flows at charge level %d: from %d to %d\n',k,c,FindRoadLinkPtckij(tt,c,k,is,RoadGraph{is}(1)),FindRoadLinkPtckij(tt,c,k,it,RoadGraph{it}(end)))
        end
        fprintf('Rebalancing flows at charge level %d: from %d to %d\n',c,FindRoadLinkRtcij(tt,c,is,RoadGraph{is}(1)),FindRoadLinkRtcij(tt,c,it,RoadGraph{it}(end)))
    end
    for c=1:C
        for k=1:M
            fprintf('Passenger %d recharging flow at charge level %d: from %d to %d\n',k,c, FindChargeLinkPtckl(tt,c,k,1),FindChargeLinkPtckl(tt,c,k,NumChargers))
        end
        fprintf('Rebalancing recharging flow at charge level %d: from %d to %d\n',c, FindChargeLinkRtcl(tt,c,1),FindChargeLinkRtcl(tt,c,NumChargers))
    end
    for c=1:C
        for k=1:M
            fprintf('Passenger %d discharging flow at charge level %d: from %d to %d\n',k,c, FindDischargeLinkPtckl(tt,c,k,1),FindDischargeLinkPtckl(tt,c,k,NumChargers))
        end
        fprintf('Rebalancing discharging flow at charge level %d: from %d to %d\n',c, FindDischargeLinkRtcl(tt,c,1),FindDischargeLinkRtcl(tt,c,NumChargers))
    end
    for c=1:C
        fprintf('Source flows for all pax at charge level %d: %d to %d\n',c,FindPaxSourceChargecks(c,1,1),FindPaxSourceChargecks(c,M,length(Sources{M})));
    end
    for c=1:C
        fprintf('Sink flows for all pax at charge level %d and time %d: %d to %d\n',c,1,FindPaxSinkChargetck(1,c,1),FindPaxSinkChargetck(1,c,M));
    end
    for c=1:C
        fprintf('Sink flows for all pax at charge level %d and time %d: %d to %d\n',c,Thor,FindPaxSinkChargetck(Thor,c,1),FindPaxSinkChargetck(Thor,c,M));
    end
    fprintf('Next block of entries starts at %d\n',FindRoadLinkPtckij(tt+1,1,1,is,RoadGraph{is}(1)))
    
    fprintf('End location of empty vehicles: %d to %d\n',FindEndRebLocationci(1,1),FindEndRebLocationci(C,N))
    
    if congrelaxflag
        fprintf('Cong. relax entries: %d to %d (there should be %d entries)\n',FindCongRelaxtij(tt,is,RoadGraph{is}(1)),FindCongRelaxtij(tt,it,RoadGraph{it}(end)),E)
    end
    if sourcerelaxflag
        fprintf('Source relax entries: %d to %d (there should be %d entries)\n',FindSourceRelaxks(1,1),FindSourceRelaxks(M,length(Sources{M})),M)
    end
%     
     for tt=[1 Thor];
         fprintf('Generator flows at time %d: %d to %d\n',tt,FindGeneratorti(tt,1),FindGeneratorti(tt,NumGenerators))
         
         fprintf('Phase angles flows at time %d for electric nodes: %d to %d\n',tt,FindPhaseAngleti(tt,1),FindPhaseAngleti(tt,NP))
         
         fprintf('Link flows at time %d: %d to %d\n',tt,FindPowerLinktij(tt,1,1),FindPowerLinktij(tt,NP,length(PowerGraphM{NP})))
         
     end
     if powernetworkloadrelaxflag
         fprintf('Power network load relax entries: %d to %d (there should be %d entries)\n',FindPowerNetworkRelaxti(1,1),FindPowerNetworkRelaxti(Thor,NP),Thor*NP )
     end
end