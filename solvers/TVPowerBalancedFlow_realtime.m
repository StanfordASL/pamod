function [cplex_out,fval,exitflag,output,lambdas,dual_prices_ix,dual_charger_prices_ix,lb,solveTime,lp_matrices]=TVPowerBalancedFlow_realtime(Thor,RoadNetwork,PowerNetwork,InitialConditions,RebWeight,Passengers,Flags)

% A real-time version of TVPowerBalancedFlow_withpower_bundle.
% This version only computes routes for rebalancing vehicles: customer
% routes are assumed to be fixed, i.e. customers travel along the fastest
% available route. 
% Supports multi-edges in the power network. Expects customer requests to
% be bundled by sink (see below for input format). Compared to the
% non-real-time version, this function expects two extra entries in theMinNumVehiclesAti
% RoadNetwork struct encoding the charge level and travel time required to
% travel between nodes. Also, the RoadCap matrix is now time-varying to
% account for customers' travel patterns.
%
% Function to solve time-varying AMoD routing and rebalancing with
% charging and a power network in the background. Finds the socially
% optimal vehicle routes and power generator allocation for a given set of
% customer demands and available generators (each with a price).

% Inputs:
% * Thor, the optimization horizon (an int)
% * Inside RoadNetwork
%     - C, a scalar. The overall number of charge levels.
%     - RoadGraph, a nx1 cell structure. RoadGraph{i} contains the neighbors of
%        i in the road graph
%     - TVRoadCap, a txnxn matrix. RoadCap(t,i,j) is the capacity of the i-j link
%        (in vehicles per unit time) at time t.
%     - TravelTimes, a nxn matrix of ints. TravelTimes(i,j) is the travel time
%        along the i-j link.
%     - TravelDistance, a nxn matrix of floats. TravelDistance(i,j) is the
%        travel distance along the i-j link.
%     - ChargeToTraverse, a nxn matrix. ChargeToTraverse(i,j) is the amount of (quantized)
%        units of charge required to travel from i to j. For a charge link,
%     - ChargersList, a vector that identifies chargers. CL(i) is the location
%        of the node in RoadGraph corresponding to the i-th charger
%     - ChargerSpeed, the charging speed (in charge units per time) of the
%        chargers. CS(i) is the amount of charge gained by a vehicle crossing 
%        charge link i (corresponding to node CS(i)).
%     - ChargerTime, the time required to charge ChargerSpeed units of charger
%        at charger i. CS(i) is the charge speed of charger i (at node CL(i)).
%     - ChargerCost [NOT USED], a NumChargersxThor vector representing the cost of
%        charging at a charger. CC(i,t) is the cost to charge for an unit of
%        time at charger i at time t.
%     - ChargerCap, the maximum number of vehicles that a charger can service.
%     - ChargeUnitToEnergyUnit, a scalar denoting the conversion between charge
%        units per unit time (in the AMoD problem) and energy units (in the
%        power network problem).
%     - MinEndCharge, a scalar denoting the minimum charge (in charge levels)
%        of the rebalancing vehicles at the end of the simulation.
%     - ValueOfTime, the value (in USD) of one unit of time for a passenger
%     - VehicleCostPerm, the cost of running a vehicle for 1m
%     - RouteTime, a nxn matrix of floats. RouteTime(i,j) is the shortest
%       travel time from node i to node j (along some path).
%     - RouteCharge, a nxn matrix. RouteCharge(i,j) is the charge level
%       required to travel along the shortest path from i to j.
%
% * Inside InitialConditions
%     - EmptyVehicleInitialPosRT, a ThorxNxC matrix encoding the initial
%        position and charge level of the vehicles. EVIP-RT(t,i,c) is the
%        number of empty vehicles at node i and charge c at time t. 
%     - MinNumVehiclesAti, a Nx1 vector denotingthe min. number of vehicles
%        at location i at the end time. 
%
% * RebWeight, the relative importance of the rebalancing time wrt the
%    customer cost. Used to discourage needless charging/discharging.
%
% * Inside Passengers
%     - Sinks, a m-x-1 vector. Sinks(i) is the sink node of the i-th
%        flow.
%     - Sources, a m-x-1 cell. Sources{i} is a vector. Sources{i}(j) is the
%        j-th origin leaving at time StartTimes{i}(j) for sink Sinks(i).
%     - StartTimes, same as Sources. ST{i}(j) is the departure time from
%        source Sources{i}(j).
%     - Flows, a m-by-1 cell. Flows{i}(j) is the amount of (vehicle) flow
%        entering the source of flow Sources{i}(j) at time ST{i}(j) bound for
%        Sinks(i).
%       We also model outstanding customers, who are already waiting. It is
%       sometimes too constraining to ask for them to be picked up at t=1,
%       so we leave the pickup time free.
%     - StarterSinks: a msx1 vector. SSinks(i) is the sink node of the i-th
%       starter flow.
%     - StarterSources: a ms-x-1 cell. SSources{i}(j) is the j-th origin
%       node leaving (at some time) for sink SSinks(i).
%     - StarterFlows, a ms-by-1 cell. SFlows{i}(j) is the amount of
%       vehicle flow who wants to travel from SSources{i}(j) to SSinks(i)
%       asap.
%
% * Inside PowerNetwork
%     - PowerGraph: A npx1 cell structure encoding the topology of the power
%        network graph. Analogous to RoadGraph. PowerGraph should represent an
%        UNDIRECTED graph: include either edge ij or ji, not both. Multiple
%        edges are supported.
%     - PowerLineCap: A npx1 cell structure . PLC{i}(j) is the max. capacity
%        (in W) of link PG{i}(j).
%     - PowerLineReactance: A npx1 cell structure . PLR{i}(j) is the reactance
%        of link PG{i}(j).
%     - PowerGensList: a list of the power nodes that are generators. Generator
%        g is node PGL(g) in PowerGraph.
%     - PowerGensMax: maximum power generated by these generators. PGM(t,g) is
%        the max power generated by generator g at time t
%     - PowerGensMin: minimum power generated by these generators. Analogous to
%        PGM above.
%     - PowerCosts: cost of one unit of power from these generators. PC(t,g) is
%        the cost of one unit of power from generator g at time t
%     - PowerExtLoads: a npxThor list of the (fixed) loads at the nodes.
%        These do not include AMoD loads, which are added on top.
%     - RoadToPowerMap: a NumChargersx1 vector. RTPM(l) is the node
%        in the power graph that corresponds to charger l.
%     - PowerRampUp: the maximum ramp-up rate for a generator. PRU(t,g) is the
%        maximum increase in power a generator can experience (that is,
%        P(t+1,g)<=P(t,g)+PRU(t,g)).
%     - PowerRampDown: the maximum ramp-down rate for a generator. PRD(t,g) is
%        the maximum decrease in power a generator can experience (that is,
%        P(t+1,g)>=P(t,g)-PRD(t,g).
%     - VoLL, the value of lost load. The cost of shedding one unit of load
%        if the power network is infeasible and
%        Flags.powernetworkloadrelaxflag is set to one.

% * Inside Flags
%     - milpflag (default: 1). If 1, the vehicle routing problem is solved as a
%        MILP. If 0, the problem is solved as a linear relaxation.
%     - congrelaxflag (default: 0). If this flag is on, then vehicle flows are
%        allowed to violate the  congestion constraint for a cost.
%        A slack variable is defined for each edge. The cost is defined in the
%        main code.
%     - sourcerelaxflag (default: 0). %If this flag is on, each vehicle flow is
%        allowed to reduce its sources (and sinks) for a cost. This is
%        especially useful when it is not possible to compute a satisfying
%        rebalancing flow because of timing constraints but, if the congestion
%        constraints are tight, it can also preserve feasibility if not all
%        flows can be realized. A slack variable is defined for each source and sink.
%     - solverflag (default: 'MOSEK'). Specifies the solver to use. Options: 'MOSEK'
%        and 'CPLEX'.
%     - powernetworkloadrelaxflag (default=0). If set to 1, allows the
%        power network constraints to be relaxed for feasibility.
%     - cyclicflag (default = 0). If cyclicflag == 1, allows the solution to
%        "wrap around" the time horizon and enforces that the state at Thor
%        equal the state at t=1. BROKEN.
%        If cyclicflag == 2, enforces that final condition equal initial
%        condition and ramp-up constraints on generators across time
%        horizon.
%        If cyclicflag ==, enforces that final condition equal initial
%        condition, and leaves initial SoC free (subject to constraint on
%        tot. number of vehicles.
%
%



DIAGNOSTIC_FLAG=0;  % Diagnoses state allocation. Useful for initial debugging.

try
  SOLVER_FLAG=Flags.solverflag;
catch
  SOLVER_FLAG = 'MOSEK'; % CPLEX for CPLEX, MOSEK for MOSEK
end
if ~strcmp(SOLVER_FLAG,'CPLEX') & ~strcmp(SOLVER_FLAG,'MOSEK') & ~strcmp(SOLVER_FLAG,'GLPK')
    SOLVER_FLAG = 'CPLEX';
    disp('WARNING: invalid solver, setting solver to CPLEX')
end
disp(SOLVER_FLAG);

try
    solverstoppingprecision = Flags.solverstoppingprecision;
catch
    solverstoppingprecision = 1.0e-8
end

TVPowerBalancedFlowFinder_realtime;

assert(cyclicflag~=1,'Cyclic mode is broken (number of vehicles is not enforced properly');
%%
   

%% COST
f_cost=zeros(StateSize,1);
% Passengers' and rebalancers' travel time
for t=1:Thor
    for i=1:N
        for j=RoadGraph{i}
            for c=1:C
                if j~=i
                    f_cost(FindRoadLinkRtcij(t,c,i,j))= TravelDistance(i,j)*VehicleCostPerm + TravelTimes(i,j)*ValueOfTime*RebWeight + abs(ChargeToTraverse(i,j))*BatteryDepreciationPerUnitCharge;
                end
            end
        end
    end
end

% Cost of picking up a starter pax late
for t=1:Thor
    for c=1:C
        for k=1:MS
            for ssi=1:length(StarterSources{k})
                f_cost(FindStarterSourceChargetcks(t,c,k,ssi)) = ValueOfTime*t;
            end
        end
    end
end

% Passengers' and rebalancers' charging time and cost
for t=1:Thor
    for i=1:length(ChargersList)
        for c=1:C
            f_cost(FindChargeLinkRtcl(t,c,i))=RebWeight*ValueOfTime*ChargerTime(i) + abs(ChargerSpeed(i))*BatteryDepreciationPerUnitCharge;
            f_cost(FindDischargeLinkRtcl(t,c,i))=RebWeight*ValueOfTime*ChargerTime(i) + abs(ChargerSpeed(i))*BatteryDepreciationPerUnitCharge;
        end
    end
end

% Cost of power generation
for t=1:Thor
    for g=1:NumGenerators
        try
            f_cost(FindGeneratorti(t,g))=PowerCosts(t,g);
        catch
            disp('this should not happen')
            keyboard
        end
    end
end

% Penalize discharging if v2g = 0
% if v2g_efficiency <=0
%     disp('Penalizing v2g discharging ')
%     for t=1:Thor
%         for c=1:C
%             for l=1:length(ChargersList)
%                 f_cost(FindDischargeLinkRtcl(t,c,l)) = SourceRelaxCost;
%             end
%         end
%     end
% end

% Cost of various relaxations
if congrelaxflag
    for t=1:Thor
        for i=1:N
            if ~isempty(RoadGraph{i})
                f_cost(FindCongRelaxtij(t,i,RoadGraph{i}(1)):FindCongRelaxtij(t,i,RoadGraph{i}(end)))=CongestionCost;
            end
        end
    end
end


if sourcerelaxflag
    for k=1:M
        for ssi=1:length(Sources{k})
            f_cost(FindSourceRelaxks(k,ssi))=SourceRelaxCost;
        end
    end
    for k=1:MS
        for ssi=1:length(StarterSources{k})
            f_cost(FindStarterSourceRelaxks(k,ssi))=2*SourceRelaxCost; % we arbitrarily penalize waiting pax 2x as much as future pax
        end
    end
end

if minendchargerelaxflag
    for c=1:MinEndCharge-1
        f_cost(FindChargeRelaxci(c,1):FindChargeRelaxci(c,N)) = MinEndChargeRelaxCost*(MinEndCharge-c); %penalize lower charge levels more
    end
end

if endreblocrelaxflag
   for i=1:N
       f_cost(FindEndRebLocRelaxi(i))=EndRebLocRelaxCost;
   end
end

if powernetworkloadrelaxflag
    for t=1:Thor
        for i=1:NP
            f_cost(FindPowerNetworkRelaxUPti(t,i))=PowerNetworkRelaxCost;
            f_cost(FindPowerNetworkRelaxDNti(t,i))=PowerNetworkRelaxCost;
        end
    end
end

%% INITIALIZING CONSTRAINTS
if (debugflag)
    disp('Initializing constraints')
end

% The narrative below may be outdated
% Vehicles:
% N*Thor*(1)*C +M+TotNumSources + M*T*C equality constraints, one per unit time per node
%  per flow and per charge level plus one for each source and sink plus one
%  per sink per charge level per unit time to ensure that pax (which no
%  longer have their own flow) are conserved,
%  plus  TotNumStarterSources + MS + Thor*M*C for starter sources, starter
%  sinks, starter charge conservation, in analogy with pax.
% Up to 2*(E+2*NumChargers)*Thor*(M+1)*C + 2*C*TotNumSources + 2*Thor*C*M + 2*TotNumSources entries:
%   two per link per unit time per flow per charge level plus two per flow
%   per charger pointing to the next charge level (one up, one down)
%   plus C*TotNumSources for the source flows (to guarantee that
%   sum(sources_c)=source), and Thor*C*M to guarantee that sum(sinks)=sink
%   plus C*TotNumSources + Thor*C*M to guarantee conservation.
%   Plus 2*TotNumSources for the source and sink relaxation.
%   plus (TotNumStarterSources+MS)*Thor*C (starter conservation)+
%   plus TotNumStarterSources*(Thor*C+1) (starter sources)
%   plus + MS*Thor*C+TotNumStarterSources (starter sinks) +
%   plus (TotNumStarterSources+MS)*Thor*C( starter charge conservation)
% Power:
% Power grid: Thor*(NP+EP +EP) equality constraints, one per node (power balance)
%   and one per link (phase angle) and one per symmetry.
% Thor*(EP+NumGenerators+2*NumChargers + 5*EP) entries, one per link plus one per
%   gen and two per charger load (charge-discharge) and three for each
%   phase angle link and two per link for symmetry.
%   plus NP constraints for load relaxation if needed.



n_eq_constr = N*(1)*C*Thor + M + TotNumSources + M*Thor*C + ...
    TotNumStarterSources + MS + Thor*MS*C+...
    Thor*(NP+EP);
n_eq_entries = 2*(E+2*NumChargers)*(M+1)*C*Thor + 2*C*TotNumSources+ 2*Thor*C*M + ...
    (TotNumStarterSources+MS)*Thor*C + TotNumStarterSources*(Thor*C+1) + ...
    MS*Thor*C+TotNumStarterSources + (TotNumStarterSources+MS)*Thor*C + ...
    Thor*(2*EP+NumGenerators+2*sum(ChargerTime) + 3*EP);
%TODO UPDATE AEQENTRIES AND AEQCONSTR
%TODO Entries: (TotNumStarterSources+MS)*Thor*C (conservation)+TotNumStarterSources*(Thor*C+1) (sources) + MS*Thor*C+TotNumStarterSources (sinks) + (TotNumStarterSources+MS)*Thor*C(charge conservation)
%TODO Constraints: TotNumStarterSources + MS + Thor*M*C (starter sources, starter sinks, starter charge conservation)

if sourcerelaxflag
    n_eq_entries=n_eq_entries+2*TotNumSources;
end
if powernetworkloadrelaxflag
    n_eq_entries=n_eq_entries+2*NP*Thor;
end
if cyclicflag == 1 || cyclicflag == 3
    n_eq_constr = n_eq_constr+1;
    n_eq_entries = n_eq_entries + 2*N*C;
end

Aeqsparse=zeros(n_eq_entries,3);
Beq=zeros(n_eq_constr,1);
Aeqrow=1;
Aeqentry=1;

dual_prices_ix=zeros(n_eq_constr,1);
dual_charger_prices_ix=zeros(n_eq_constr,1);

% Vehicles: E*Thor inequality constraints, one per road per unit time. Plus
%  2*NumChargers*Thor for the charge-discharge links. Plus
%  (MinEndCharge-1)*N constraints for the end charge (we handle it here so
%  we can relax)+N constraints for the end location (same as charge).
% Each road inequality constraint has C*(1) entries (one per flow,
%  incl. reb.). Chargers have Thor*C*(M+1) %  entries (no relaxation).
%  There are (MinEndCharge-1)*N entries for the final charge level, plus an
%  extra (MinEndCharge-1)*N if the relaxation is active. There are N*C
%  entries for the end location, plus N extra if we relax.
% Power network: 2*NumGenerators*(T-1) ramp-up and ramp-down constraints.
%  Thermal constraints and generators are handled in ub and lb

n_ineq_constr = (E+NumChargers)*Thor+2*NumGenerators*(Thor-1)+(MinEndCharge-1)*N+N;
n_ineq_entries = (E + 2*NumChargers)*(1)*C*Thor + 4*NumGenerators*(Thor-1)+(MinEndCharge-1)*N+N*C;
if congrelaxflag
    n_ineq_entries=n_ineq_entries+E*Thor;
end
if minendchargerelaxflag
    n_ineq_entries=n_ineq_entries + (MinEndCharge-1)*N;
end
if endreblocrelaxflag
    n_ineq_entries=n_ineq_entries+N;
end
if cyclicflag ~= 0
    n_ineq_constr = n_ineq_constr + 2*NumGenerators;
    n_ineq_entries = n_ineq_entries + 4*NumGenerators;
end

Aineqsparse=zeros(n_ineq_entries,3);
Bineq=zeros(n_ineq_constr,1);
Aineqrow=1;
Aineqentry=1;

%% EQUALITY CONSTRAINTS
if (debugflag)
    disp('Building sparse equality constraints...')
end


% Conservation of rebalancers
fprintf("  Conservation of rebalancers: ")
for t=1:Thor
    fprintf("%d/%d..",t,Thor)
    for c=1:C
        for i=1:N
            if ~cachedAeqflag
                if ~isempty(RoadGraph{i})
                    for j=RoadGraph{i} %Out-flows
                        if ((ChargeToTraverse(i,j)< c))
                            if (t+full(TravelTimes(i,j))<=Thor) || cyclicflag==1
                                Aeqsparse(Aeqentry,:)=[Aeqrow,FindRoadLinkRtcij(t,c,i,j), 1];
                                Aeqentry=Aeqentry+1;
                            end
                        end
                    end
                end
                if ~isempty(ReverseRoadGraph{i})
                    for j=ReverseRoadGraph{i} %In-flows
                        if (ChargeToTraverse(j,i)+c<=C)
                            if (t-full(TravelTimes(j,i))>0)
                                Aeqsparse(Aeqentry,:)=[Aeqrow,FindRoadLinkRtcij(t-full(TravelTimes(j,i)),ChargeToTraverse(j,i)+c,j,i),-1];
                                Aeqentry=Aeqentry+1;
                            elseif cyclicflag==1
                                Aeqsparse(Aeqentry,:)=[Aeqrow,FindRoadLinkRtcij(mod(t-full(TravelTimes(j,i)-1),Thor)+1,ChargeToTraverse(j,i)+c,j,i),-1];
                                Aeqentry=Aeqentry+1;
                            end
                        end
                    end
                end
                
                for l=1:length(ChargersList)
                    if (ChargersList(l)==i) %is a charger
                        if c+ChargerSpeed(l)<=C
                            %add link to i,c+1.
                            if (t+ChargerTime(l)<=Thor) || cyclicflag==1
                                Aeqsparse(Aeqentry,:)=[Aeqrow,FindChargeLinkRtcl(t,c,l),1];
                                Aeqentry=Aeqentry+1; %Charge up to c+1, goes out
                            end
                            if (t-ChargerTime(l)>0)
                                Aeqsparse(Aeqentry,:)=[Aeqrow,FindDischargeLinkRtcl(t-ChargerTime(l),c+ChargerSpeed(l),l),-1];
                                Aeqentry=Aeqentry+1; %Charge down from c+1, goes in
                            elseif cyclicflag==1
                                Aeqsparse(Aeqentry,:)=[Aeqrow,FindDischargeLinkRtcl(mod(t-ChargerTime(l)-1,Thor)+1,c+ChargerSpeed(l),l),-1];
                                Aeqentry=Aeqentry+1; %Charge down from c+1, goes in
                            end
                        end
                        if c-ChargerSpeed(l)>=1
                            if (t-ChargerTime(l)>0)
                                Aeqsparse(Aeqentry,:)=[Aeqrow,FindChargeLinkRtcl(t-ChargerTime(l),c-ChargerSpeed(l),l),-1];
                                Aeqentry=Aeqentry+1; %Charge up from c-1, goes in
                            elseif cyclicflag==1
                                Aeqsparse(Aeqentry,:)=[Aeqrow,FindChargeLinkRtcl(mod(t-ChargerTime(l)-1,Thor)+1,c-ChargerSpeed(l),l),-1];
                                Aeqentry=Aeqentry+1; %Charge up from c-1, goes in                                
                            end
                            if (t+ChargerTime(l)<=Thor) || cyclicflag==1
                                Aeqsparse(Aeqentry,:)=[Aeqrow,FindDischargeLinkRtcl(t,c,l),1];
                                Aeqentry=Aeqentry+1; %Charge down to c-1, goes out
                            end
                        end
                    end
                end
                
                for k=1:M
                    for ssi=1:length(Sources{k})
                        if (Sources{k}(ssi)==i && StartTimes{k}(ssi)==t)
                            Aeqsparse(Aeqentry,:)=[Aeqrow,FindPaxSourceChargecks(c,k,ssi),1];
                            Aeqentry=Aeqentry+1;
                            % Departing passengers (exiting vehicles) at charge level c
                        end
                    end
                    if (Sinks(k)==i)
                        Aeqsparse(Aeqentry,:)=[Aeqrow,FindPaxSinkChargetck(t,c,k),-1];
                        Aeqentry=Aeqentry+1;
                        %Arriving passengers (entering vehicles)
                    end
                    
                end
                for k=1:MS
                    for ssi=1:length(StarterSources{k})
                        if StarterSources{k}(ssi)==i
                            Aeqsparse(Aeqentry,:)=[Aeqrow,FindStarterSourceChargetcks(t,c,k,ssi),1];
                            Aeqentry=Aeqentry+1;
                        end
                    end
                    if StarterSinks(k)==i
                        Aeqsparse(Aeqentry,:)=[Aeqrow,FindStarterSinkChargetck(t,c,k),-1];
                        Aeqentry=Aeqentry+1;
                    end
                end
                % Final conditions
                if t==Thor
                    Aeqsparse(Aeqentry,:)=[Aeqrow,FindEndRebLocationci(c,i),1];
                    Aeqentry=Aeqentry+1;
                end
                if t==1 && (cyclicflag==1 || cyclicflag==3)
                    Aeqsparse(Aeqentry,:)=[Aeqrow,FindEndRebLocationci(c,i),-1];
                    Aeqentry=Aeqentry+1;
                end
            end
            % Initial conditions!
            if (cyclicflag~=1 && cyclicflag ~=3)
                Beq(Aeqrow)=EmptyVehicleInitialPosRT(t,i,c);
            else
                Beq(Aeqrow) = 0;
            end
            Aeqrow=Aeqrow+1;
        end
    end
end
    
if cyclicflag == 1 || cyclicflag == 3
    for c=1:C
        for i=1:N
            Aeqsparse(Aeqentry,:)=[Aeqrow,FindEndRebLocationci(c,i),1];
            Aeqentry=Aeqentry+1;
        end
    end
    Beq(Aeqrow) = sum(sum(sum(EmptyVehicleInitialPosRT))); %The total correct number of vehicles is out there.
    Aeqrow = Aeqrow+1;
end
fprintf("\n")

% Sum of all FindPaxSourceChargeck = Pax. source

for k=1:M
    for ssi=1:length(Sources{k})
        if ~cachedAeqflag
            for c=1:C
                Aeqsparse(Aeqentry,:)=[Aeqrow,FindPaxSourceChargecks(c,k,ssi),1];
                Aeqentry=Aeqentry+1;
            end
            if sourcerelaxflag
                Aeqsparse(Aeqentry,:)=[Aeqrow,FindSourceRelaxks(k,ssi),1];
                Aeqentry=Aeqentry+1;
            end
        end
        Beq(Aeqrow)=Flows{k}(ssi);
        Aeqrow=Aeqrow+1;
    end
end

% Sum of all FindPaxSinkChargeck = Pax. sink
for k=1:M
    if ~cachedAeqflag
        for c=1:C
            for t=1:Thor
                Aeqsparse(Aeqentry,:)=[Aeqrow,FindPaxSinkChargetck(t,c,k),1];
                Aeqentry=Aeqentry+1;
            end
        end
        if sourcerelaxflag
            for ssi=1:length(Sources{k})
                Aeqsparse(Aeqentry,:)=[Aeqrow,FindSourceRelaxks(k,ssi),1];
                Aeqentry=Aeqentry+1;
            end
        end
    end
    Beq(Aeqrow)=sum(Flows{k});
    Aeqrow=Aeqrow+1;
end


% Sum of all FindStarterSourceChargeck = Pax. source
for k=1:MS
    for ssi=1:length(StarterSources{k})
        if ~cachedAeqflag
            for t=1:Thor
                for c=1:C
                    Aeqsparse(Aeqentry,:)=[Aeqrow,FindStarterSourceChargetcks(t,c,k,ssi),1];
                    Aeqentry=Aeqentry+1;
                end
            end
            if sourcerelaxflag
                Aeqsparse(Aeqentry,:)=[Aeqrow,FindStarterSourceRelaxks(k,ssi),1];
                Aeqentry=Aeqentry+1;
            end
        end
        Beq(Aeqrow)=StarterFlows{k}(ssi);
        Aeqrow=Aeqrow+1;
    end
end

% Sum of all FindStarterSinkChargeck = Pax. sink
for k=1:MS
    if ~cachedAeqflag
        for c=1:C
            for t=1:Thor
                Aeqsparse(Aeqentry,:)=[Aeqrow,FindStarterSinkChargetck(t,c,k),1];
                Aeqentry=Aeqentry+1;
            end
        end
        if sourcerelaxflag
            for ssi=1:length(StarterSources{k})
                Aeqsparse(Aeqentry,:)=[Aeqrow,FindStarterSourceRelaxks(k,ssi),1];
                Aeqentry=Aeqentry+1;
            end
        end
    end
    Beq(Aeqrow)=sum(StarterFlows{k});
    Aeqrow=Aeqrow+1;
end

% Conservation of customer charge
for k=1:M
    for t=1:Thor
        for c=1:C
            if ~cachedAeqflag
                Aeqsparse(Aeqentry,:)=[Aeqrow,FindPaxSinkChargetck(t,c,k),-1];
                Aeqentry=Aeqentry+1;
                for ssi=1:length(Sources{k})
                    if StartTimes{k}(ssi)==t-RouteTime(Sources{k}(ssi),Sinks(k)) || ( cyclicflag==1 && StartTimes{k}(ssi)==mod(t-RouteTime(Sources{k}(ssi),Sinks(k))-1,Thor)+1)
                        if c+RouteCharge(Sources{k}(ssi),Sinks(k))<=C && c+RouteCharge(Sources{k}(ssi),Sinks(k))>0
                            Aeqsparse(Aeqentry,:)=[Aeqrow,FindPaxSourceChargecks(c+RouteCharge(Sources{k}(ssi),Sinks(k)),k,ssi),1];
                            Aeqentry=Aeqentry+1;
                        end
                    end
                end
            end
            Beq(Aeqrow)=0;
            Aeqrow=Aeqrow+1;
        end
    end
end


% Conservation of starter customer charge
for k=1:MS
    for t=1:Thor
        for c=1:C
            if ~cachedAeqflag
                Aeqsparse(Aeqentry,:)=[Aeqrow,FindStarterSinkChargetck(t,c,k),-1];
                Aeqentry=Aeqentry+1;
                for ssi=1:length(StarterSources{k})
                    tti=t-RouteTime(StarterSources{k}(ssi),StarterSinks(k));
                    if tti>0 && tti<=Thor || cyclicflag==1
                        tti = mod(tti-1,Thor)+1; %This has no effect in case 1 and the desired effect if cyclicglag is set
                        if c+RouteCharge(StarterSources{k}(ssi),StarterSinks(k))<=C && c+RouteCharge(StarterSources{k}(ssi),StarterSinks(k))>0
                            Aeqsparse(Aeqentry,:)=[Aeqrow,FindStarterSourceChargetcks(tti,c+RouteCharge(StarterSources{k}(ssi),StarterSinks(k)),k,ssi),1];
                            Aeqentry=Aeqentry+1;
                        end
                    end
                end
            end
            Beq(Aeqrow)=0;
            Aeqrow=Aeqrow+1;
        end
    end
end


%% 
% Power grid: conservation of power
fprintf("  Conservation of power ")
for t=1:Thor
    fprintf("%d/%d..",t,Thor)
    for i=1:NP
        if ~cachedAeqflag
            if ~isempty(PowerGraphM{i})
                for j=1:length(PowerGraphM{i})
                    % Outbound power - outbound is positive
                    Aeqsparse(Aeqentry,:)=[Aeqrow,FindPowerLinktij(t,i,j),1];
                    Aeqentry=Aeqentry+1;
                end
            end
            for l=1:NP
                for p=1:length(PowerGraphM{l})
                    if PowerGraphM{l}(p)==i
                        Aeqsparse(Aeqentry,:)=[Aeqrow,FindPowerLinktij(t,l,p),-1];
                        Aeqentry=Aeqentry+1;
                    end
                end
            end
            for g=1:NumGenerators
                if PowerGensList(g)==i
                    Aeqsparse(Aeqentry,:)=[Aeqrow,FindGeneratorti(t,g),-1];
                    Aeqentry=Aeqentry+1;
                    % Add gen power
                end
            end
            
            
            for l=1:NumChargers %Go through chargers in the road network
                if RoadToPowerMap(l)==i %If this node i in the power network corresponds to the charger
                    for c=1:C
                        if c+ChargerSpeed(l)<=C
                            for deltat=0:ChargerTime(l)-1
                                if t-deltat>0 && t-deltat+ChargerTime(l)<=Thor
                                    Aeqsparse(Aeqentry,:)=[Aeqrow,FindChargeLinkRtcl(t-deltat,c,l),double(ChargerSpeed(l))/double(ChargerTime(l))*double(ChargeUnitToMW)];
                                    Aeqentry=Aeqentry+1;
                                elseif cyclicflag==1
                                    Aeqsparse(Aeqentry,:)=[Aeqrow,FindChargeLinkRtcl(mod(t-deltat-1,Thor)+1,c,l),double(ChargerSpeed(l))/double(ChargerTime(l))*double(ChargeUnitToMW)];
                                end
                            end
                        end
                    end
                    for c=1:C
                        if c-ChargerSpeed(l)>=1
                            for deltat=0:ChargerTime(l)-1
                                if t-deltat>0 && t-deltat+ChargerTime(l)<=Thor
                                    Aeqsparse(Aeqentry,:)=[Aeqrow,FindDischargeLinkRtcl(t-deltat,c,l),-double(v2g_efficiency)*double(ChargerSpeed(l))/double(ChargerTime(l))*double(ChargeUnitToMW)];
                                    Aeqentry=Aeqentry+1;
                                elseif cyclicflag==1
                                    Aeqsparse(Aeqentry,:)=[Aeqrow,FindDischargeLinkRtcl(mod(t-deltat-1,Thor)+1,c,l),-double(v2g_efficiency)*double(ChargerSpeed(l))/double(ChargerTime(l))*double(ChargeUnitToMW)];
                                    Aeqentry=Aeqentry+1;
                                end
                            end
                        end
                    end
                    % Add power draw from the relevant charger
                    dual_charger_prices_ix(Aeqrow)=1;
                end
            end
            if powernetworkloadrelaxflag
                Aeqsparse(Aeqentry,:)=[Aeqrow,FindPowerNetworkRelaxUPti(t,i),-1];
                Aeqentry=Aeqentry+1;
                Aeqsparse(Aeqentry,:)=[Aeqrow,FindPowerNetworkRelaxDNti(t,i),1];
                Aeqentry=Aeqentry+1;
            end
        end
        Beq(Aeqrow)=-PowerExtLoads(i,t);
        dual_prices_ix(Aeqrow)=1;
        Aeqrow=Aeqrow+1;
        % Add base power on B
    end
end
fprintf("\n")
%%
%
% Power grid: phase angle
for t=1:Thor
    for i=1:NP
        if ~isempty(PowerGraphM{i})
            for j=1:length(PowerGraphM{i})
                if ~cachedAeqflag
                    Aeqsparse(Aeqentry,:)=[Aeqrow,FindPowerLinktij(t,i,j),-double(full(PowerLineReactanceM{i}(j)))];
                    Aeqentry=Aeqentry+1;
                    Aeqsparse(Aeqentry,:)=[Aeqrow,FindPhaseAngleti(t,i),1];
                    Aeqentry=Aeqentry+1;
                    Aeqsparse(Aeqentry,:)=[Aeqrow,FindPhaseAngleti(t,PowerGraphM{i}(j)),-1];
                    Aeqentry=Aeqentry+1;
                end
                Beq(Aeqrow)=0;
                Aeqrow=Aeqrow+1;
            end
        end
    end
end

%% INEQUALITY CONSTRAINTS
if debugflag
    disp('Building sparse inequality constraints...')
end

% Roads: congestion
for t=1:Thor
    for i=1:N
        for j=RoadGraph{i}
            for c=1:C
                Aineqsparse(Aineqentry,:)=[Aineqrow,FindRoadLinkRtcij(t,c,i,j),1];
                Aineqentry=Aineqentry+1;
            end
            
            if congrelaxflag
                Aineqsparse(Aineqentry,:)=[Aineqrow,FindCongRelaxtij(t,i,j),-1];
                Aineqentry=Aineqentry+1;
            end
            
            Bineq(Aineqrow)=TVRoadCap(t,i,j);
            Aineqrow=Aineqrow+1;
        end
    end
end

% Chargers: congestion

for t=1:Thor
    for l=1:NumChargers
        for c=1:C
            Aineqsparse(Aineqentry,:)=[Aineqrow,FindChargeLinkRtcl(t,c,l),1];
            Aineqentry=Aineqentry+1;
            Aineqsparse(Aineqentry,:)=[Aineqrow,FindDischargeLinkRtcl(t,c,l),1];
            Aineqentry=Aineqentry+1;
        end
        Bineq(Aineqrow)=ChargerCap(l);
        Aineqrow=Aineqrow+1;
        
    end
end


% Power grid: thermal capacities now in ubs and lbs

%Power grid: ramp-up and ramp-down
for t=1:Thor-1
    for g=1:NumGenerators
        Aineqsparse(Aineqentry,:)=[Aineqrow,FindGeneratorti(t+1,g),1];
        Aineqentry=Aineqentry+1;
        Aineqsparse(Aineqentry,:)=[Aineqrow,FindGeneratorti(t,g),-1];
        Aineqentry=Aineqentry+1;
        Bineq(Aineqrow)=PowerRampUp(t,g);
        Aineqrow=Aineqrow+1;
    end
end

for t=1:Thor-1
    for g=1:NumGenerators
        Aineqsparse(Aineqentry,:)=[Aineqrow,FindGeneratorti(t+1,g),-1];
        Aineqentry=Aineqentry+1;
        Aineqsparse(Aineqentry,:)=[Aineqrow,FindGeneratorti(t,g),1];
        Aineqentry=Aineqentry+1;
        Bineq(Aineqrow)=PowerRampDown(t,g);
        Aineqrow=Aineqrow+1;
    end
end

if cyclicflag~=0
    for g=1:NumGenerators
        Aineqsparse(Aineqentry,:)=[Aineqrow,FindGeneratorti(1,g),1];
        Aineqentry=Aineqentry+1;
        Aineqsparse(Aineqentry,:)=[Aineqrow,FindGeneratorti(Thor,g),-1];
        Aineqentry=Aineqentry+1;
        Bineq(Aineqrow)=PowerRampUp(Thor,g);
        Aineqrow=Aineqrow+1;
    end
    for g=1:NumGenerators
        Aineqsparse(Aineqentry,:)=[Aineqrow,FindGeneratorti(1,g),-1];
        Aineqentry=Aineqentry+1;
        Aineqsparse(Aineqentry,:)=[Aineqrow,FindGeneratorti(Thor,g),1];
        Aineqentry=Aineqentry+1;
        Bineq(Aineqrow)=PowerRampDown(Thor,g);
        Aineqrow=Aineqrow+1;
    end
end

% Vehicles: Minimum end charge
for i=1:N
    for c=1:MinEndCharge-1
        Aineqsparse(Aineqentry,:)=[Aineqrow,FindEndRebLocationci(c,i),1];
        Aineqentry=Aineqentry+1;
        if minendchargerelaxflag
            Aineqsparse(Aineqentry,:)=[Aineqrow,FindChargeRelaxci(c,i),-1];
            Aineqentry=Aineqentry+1;
        end
        Bineq(Aineqrow)=0;
        Aineqrow=Aineqrow+1;
    end
end

% Vehicles: Minimum number at end location. N constraints, N*C + (if
% endreblocrelaxflag)(N) entries.
for i=1:N
    for c=1:C
        Aineqsparse(Aineqentry,:)=[Aineqrow,FindEndRebLocationci(c,i),-1];
        Aineqentry=Aineqentry+1;
    end
    if endreblocrelaxflag
        Aineqsparse(Aineqentry,:)=[Aineqrow,FindEndRebLocRelaxi(i),-1];
        Aineqentry=Aineqentry+1;
    end
    Bineq(Aineqrow)=-MinNumVehiclesAti(i);
    Aineqrow=Aineqrow+1;
end


%% Make equality and inequality matrices

if Aeqrow-1~=n_eq_constr
    fprintf('ERROR: unexpected number of equality constraints (expected: %d, actual: %d)\n',n_eq_constr,Aeqrow-1)
end
if Aeqentry-1~=n_eq_entries
    fprintf('Warning: unexpected number of equality entries (expected: %d, actual: %d)\n',n_eq_entries,Aeqentry-1)
end

if Aineqrow-1~=n_ineq_constr
    disp('ERROR: unexpected number of inequality constraints')
end

if Aineqentry-1~=n_ineq_entries
    fprintf('ERROR: unexpected number of inequality entries (expected: %d, actual: %d)\n',n_ineq_entries,Aineqentry-1)
end

if (debugflag)
    disp('Building matrices from sparse representation')
end


if ~cachedAeqflag
    Aeqsparse=double(Aeqsparse);
    Aeqsparse=Aeqsparse(1:Aeqentry-1,:);
    Aeqsparse=double(Aeqsparse);
    Aineqsparse = double(Aineqsparse);
    Aineqsparse=Aineqsparse(1:Aineqentry-1,:);
    Aineqsparse = double(Aineqsparse);
    Bineq=Bineq(1:Aineqrow-1,:);
    dual_prices_ix=dual_prices_ix(1:Aeqrow-1);
    dual_charger_prices_ix=dual_charger_prices_ix(1:Aeqrow-1);
    Aeq=sparse(Aeqsparse(:,1),Aeqsparse(:,2),Aeqsparse(:,3), Aeqrow-1, StateSize);
    save('Aeq_TV_RT_save.mat','Aeq','dual_prices_ix','dual_charger_prices_ix','-v7.3')
else
    load('Aeq_TV_RT_save.mat')
end

Aineq=sparse(Aineqsparse(:,1),Aineqsparse(:,2),Aineqsparse(:,3), Aineqrow-1, StateSize);

%% Upper and lower bounds
if (debugflag)
    disp('Building upper and lower bounds')
end

lb=zeros(StateSize,1); %Passenger and rebalancing flows, passenger sources and sinks,...
%                       generator loads, phase angles, slack variables
%                       can't be negative. Electric flows can be (since
%                       they are symmetric, they will, indeed)
ub=Inf*ones(StateSize,1); %Why not? We enforce capacity separately

% We have to be careful with discharging constraints: in particular, we
% should pin discharging links that do not appear in the equalities to
% zero.
% ChargeLink(t,c,..) starts at time t and goes up. So we exclude it if
% either t+ChargerTime>Thor or c+ChargerSpeed>C.
% DischargeLink(t,c,...) starts at time t and goes down. So we exclude it
% if either t+ChargerTime>Thor or c-ChargerSpeed<1.
for l=1:NumChargers
    for c=1:C
        for t=1:Thor
            if t+ChargerTime(l)>Thor && cyclicflag ~= 1
                %add link to i,c+1.
                ub(FindChargeLinkRtcl(t,c,l))=0;
                ub(FindDischargeLinkRtcl(t,c,l))=0;
            end
            if c+ChargerSpeed(l)>C
                ub(FindChargeLinkRtcl(t,c,l))=0;
            end
            if c-ChargerSpeed(l)<1
                ub(FindDischargeLinkRtcl(t,c,l))=0;
            end
        end
    end
end

for t=1:Thor
    for i=1:NP
        if ~isempty(PowerGraphM{i})
            for j=1:length(PowerGraphM{i})
                lb(FindPowerLinktij(t,i,j))=-PowerLineCapM{i}(j);      %Power links are bidirectional
                ub(FindPowerLinktij(t,i,j))=PowerLineCapM{i}(j);
            end
        end
    end
end

for t=1:Thor
    for g=1:NumGenerators
        lb(FindGeneratorti(t,g))=PowerGensMin(t,g); %Generators can, in principle, do regulation down
        ub(FindGeneratorti(t,g))=PowerGensMax(t,g);
    end
end

% Don't create chargers/dischargers from thin air
for t=1:Thor
    for l=1:NumChargers
        for c=1:ChargerSpeed(l)
            ub(FindDischargeLinkRtcl(t,c,l))=0;
        end
        for c=C-ChargerSpeed(l)+1:C
            ub(FindChargeLinkRtcl(t,c,l))=0;
        end
    end
end

for l=1:NumChargers
    for t=Thor-ChargerTime(l)+1:Thor
        for c=1:C
            ub(FindChargeLinkRtcl(t,c,l))=0;
            ub(FindDischargeLinkRtcl(t,c,l))=0;
        end
    end
end

% Number of relaxed pax should never be negative
if sourcerelaxflag
    for k=1:M
        for ssi=1:length(Sources{k})
            ub(FindSourceRelaxks(k,ssi))=Flows{k}(ssi);
            lb(FindSourceRelaxks(k,ssi))=0;
        end
    end
end

% If cyclic, end location should equal initial location
 if cyclicflag==2
     for c=1:C
         for i=1:N
             ub(FindEndRebLocationci(c,i)) = EmptyVehicleInitialPosRT(1,i,c);
             lb(FindEndRebLocationci(c,i)) = EmptyVehicleInitialPosRT(1,i,c);
         end
     end
 end
 
lp_matrices.f_cost = f_cost;
lp_matrices.Aineq = Aineq;
lp_matrices.Bineq = Bineq;
lp_matrices.Aeq = Aeq;
lp_matrices.Beq = Beq;
lp_matrices.lb = lb;
lp_matrices.ub = ub;
% SourceRelaxCost is hardcoded in TVPowerBalancedFlowFinder_realtime
lp_matrices.SourceRelaxCost = SourceRelaxCost;

%% Call optimizer

if (debugflag)
    disp('Calling optimizer')
end

% tolerance
%tolerance=1e-3;
%options = cplexoptimset('cplex');
%options.simplex.tolerances.feasibility = tolerance;
%options.simplex.tolerances.optimality = tolerance;
if debugflag
    fprintf('Nonzero elements of Aineq: %d\n',nnz(Aineq))
    fprintf('Nonzero elements of Aeq: %d\n',nnz(Aeq))
end


if strcmp (SOLVER_FLAG ,'MOSEK')
    %% MOSEK format
    %Amat = [Aeq;Aineq];
    %Bup = [Beq;Bineq];
    %Blo = [Beq;-Inf*ones(size(Bineq))];
    %prob.c = f_cost;
    %prob.a = Amat;
    %prob.blc = Blo;
    %prob.buc = Bup;
    %prob.blx = lb;
    %prob.bux = ub;
    %param    = [];
    %[r,res]  = mosekopt('symbcon');
    %sc       = res.symbcon;
    %param.MSK_IPAR_INTPNT_BASIS   = 'MSK_OFF';
    %param.MSK_IPAR_OPTIMIZER = sc.MSK_OPTIMIZER_FREE_SIMPLEX;
    %param.MSK_IPAR_SIM_NETWORK_DETECT = 0;
    options = mskoptimset('');
    options = mskoptimset(options,'Diagnostics','on','MSK_IPAR_INTPNT_BASIS','MSK_BI_NEVER','MSK_DPAR_INTPNT_TOL_REL_GAP',solverstoppingprecision);
    tic
    %[cplex_out,fval,exitflag,output,lambdas]=cplexlp(f_cost,Aineq,Bineq,Aeq,Beq,lb,ub);%, [], options);
    [cplex_out,fval,exitflag,output,lambdas]=linprog(f_cost,Aineq,Bineq,Aeq,Beq,lb,ub, [], options);
    %[r,res] =  mosekopt('minimize',prob,param);
    solveTime=toc
    %disp('Untested output, may crash')
    %cplex_out = res.sol.bas;
    %fval = res.sol.pobjval;
    %exitflag=r;
    %output=res.sol.prosta;
    %lambdas.eqlin=res.sol.slc(1:Aeqrow-1)+res.sol.suc(1:Aeqrow-1)
    
end

if strcmp (SOLVER_FLAG ,'CPLEX')
    % CPLEX stuff
    options = cplexoptimset('Display', 'off', 'Algorithm', 'interior-point');
    options.barrier.crossover  = -1;
    options.barrier.limits.objrange = 1e50;
    tic
    [cplex_out,fval,exitflag,output,lambdas]=cplexlp(f_cost,Aineq,Bineq,Aeq,Beq,lb,ub,[], options);
    solveTime=toc
end

if strcmp (SOLVER_FLAG ,'GLPK')
    % CPLEX stuff
    options.lpsolver = 2; %Interior point
    ctype = [repmat('U',1,size(Aineq,1)),repmat('S',1,size(Aeq,1))];
    vartype = repmat('C',1,size(f_cost,1));
    sense = 1; %Minimize
    tic
    [cplex_out,fval,exitflag,output]=glpk(f_cost,[Aineq;Aeq],[Bineq;Beq],lb,ub,ctype, vartype,sense,options);
    lambdas = output.lambda;
    solveTime=toc
end

if (debugflag)
    fprintf('Solved! fval: %f\n', fval)
    disp(output)
    %fval
    disp(exitflag)
    %output
end


% dual_charger_prices_ix is NOT what we think it is. It is OK for aggregate
% counts, but the returned values are not in order of chargers. To prevent
% and detect improper use, we break it. We do not remove it from the set of
% returned values to avoid breaking caller functions.
dual_charger_prices_ix = NaN;

%cplex_out=-1;
