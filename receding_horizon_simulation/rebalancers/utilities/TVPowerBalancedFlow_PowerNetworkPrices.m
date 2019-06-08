function [GenLoads,PowerPrices,ChargerPrices,PowerNetworkViolation,DroppedPowerUP,DroppedPowerDN] = TVPowerBalancedFlow_PowerNetworkPrices(t,settings,RoadNetwork,LinkTime,PowerNetwork,PreviousPowerGens,ChargersList,ChargersLoad)

PNThor=1;
dt = settings.dt;
try
    ChargeVoLL = settings.ChargeVoLL;
catch
    ChargeVoLL = 1;
    disp('Charging value of lost load (VoLL) to both supply and demand')
end
    
PowerNetworkViolation = 0;

mytime = max(round(t*dt/settings.predictionTimeStep),1);

PNRoadNetwork=RoadNetwork;
PNRoadNetwork.TVRoadCap=zeros(PNThor,RoadNetwork.N,RoadNetwork.N);
PNRoadNetwork.TVRoadCap(PNThor,:,:)=PNRoadNetwork.RoadCap;
PNRoadNetwork.TravelTimes=max(round(LinkTime/settings.predictionTimeStep),LinkTime>0);
PNRoadNetwork.TravelDistance=RoadNetwork.LinkLength/1e3;

PNPowerNetwork=PowerNetwork;
PNPowerNetwork.PowerGensMax=PowerNetwork.PowerGensMax(mytime,:);
PNPowerNetwork.PowerGensMin=PowerNetwork.PowerGensMin(mytime,:);
if t==1
    PreviousPowerGens=zeros(1,length(PNPowerNetwork.PowerGensList));
else
    PNPowerNetwork.PowerGensMax=min(PowerNetwork.PowerGensMax(mytime,:),PreviousPowerGens+PowerNetwork.PowerRampUp(mytime,:)*dt/settings.predictionTimeStep);
    PNPowerNetwork.PowerGensMin=max(PowerNetwork.PowerGensMin(mytime,:),PreviousPowerGens-PowerNetwork.PowerRampDown(mytime,:)*dt/settings.predictionTimeStep);
end
if sum(PNPowerNetwork.PowerGensMax<PNPowerNetwork.PowerGensMin)
    PowerNetworkViolation = 0.5; %Trouble in a previous time step, probabiy a generator pushed to infeasibility
    PNPowerNetwork.PowerGensMin = min(PNPowerNetwork.PowerGensMax,PNPowerNetwork.PowerGensMin);
end

PNPowerNetwork.PowerCosts=PowerNetwork.PowerCosts(mytime,:);
PNPowerNetwork.PowerExtLoads=PowerNetwork.PowerExtLoads(:,mytime);
%fprintf('WARNING: Power network loads have no noise: consider adding some!\n');
% Adding noise to external loads
PNPowerNetwork.PowerExtLoads = PNPowerNetwork.PowerExtLoads.*(ones(size(PNPowerNetwork.PowerExtLoads))+settings.powerStdDev*randn(size(PNPowerNetwork.PowerExtLoads)));


for g=1:length(ChargersList)
    PNPowerNetwork.PowerExtLoads(PowerNetwork.RoadToPowerMap(g))=PNPowerNetwork.PowerExtLoads(PowerNetwork.RoadToPowerMap(g))+ChargersLoad(g);
end

%Ramp-up is handled elsewhere above
PNPowerNetwork.PowerRampUp=Inf*ones(PNThor,length(PowerNetwork.PowerGensList));
PNPowerNetwork.PowerRampDown=Inf*ones(PNThor,length(PowerNetwork.PowerGensList));

PNInitialConditions.EmptyVehicleInitialPosRT=zeros(PNThor,PNRoadNetwork.N,PNRoadNetwork.C);
PNInitialConditions.MinNumVehiclesAti=zeros(length(PNRoadNetwork.RoadGraph),1);

PNRebWeight=0;

%Just to have non-zero sink length
PNPassengers.StarterSinks=[1];
PNPassengers.StarterSources={[1]};
PNPassengers.StarterFlows={[0]};
PNPassengers.Sinks=[1];
PNPassengers.Sources={[1]};
PNPassengers.StartTimes={[PNThor]};
PNPassengers.Flows={[0]};

PNFlags.milpflag=0;
PNFlags.congrelaxflag=1;
PNFlags.sourcerelaxflag=1;
PNFlags.cachedAeqflag=0;%1-RebOptions.firstRebalancerCall;
PNFlags.debugflag=0;
PNFlags.minendchargerelaxflag=1;
PNFlags.endreblocrelaxflag=0;
PNFlags.powernetworkloadrelaxflag=0;
PNFlags.solverflag = 'MOSEK';

if strcmp(PNFlags.solverflag,'MOSEK')
    exit_error_flag = -2; %MOSEK says that for exitflag<0 "The problem is likely to be either primal or dual infeasible". However, it uses -1 for MSK_RES_TRM_STALL, which is good enough for us.
else  
    exit_error_flag = -2; %CPLEX uses -1 for numerical problems
end

[cplex_out,fval,exitflag,output,lambdas,dual_prices_ix,dual_charger_prices_ix,lb]=TVPowerBalancedFlow_realtime(PNThor,PNRoadNetwork,PNPowerNetwork,PNInitialConditions,PNRebWeight,PNPassengers,PNFlags);

[~,PowerPricesTemp] = TVPowerBalancedFlow_getpowerdata(cplex_out,lambdas,dual_prices_ix,dual_charger_prices_ix,PNThor,PNRoadNetwork,PNPowerNetwork,PNInitialConditions,PNRebWeight,PNPassengers,PNFlags);

try
    assert(exitflag>exit_error_flag,'ERROR: power network is infeasible!')
    assert(max(PowerPricesTemp)<=PNPowerNetwork.VoLL,'ERROR: max power price is way too high');
    assert(min(PowerPricesTemp)>=-PNPowerNetwork.VoLL,'ERROR: min power price is way too negative');
catch
    fprintf('\n\nTROUBLE with the power network solution (exit flag %d)!! Or perhaps weird prices. Disconnecting loads\n\n',exitflag)
    PNFlags.powernetworkloadrelaxflag=1;
    PowerNetworkViolation = 1;
    [cplex_out,fval,exitflag,output,lambdas,dual_prices_ix,dual_charger_prices_ix,lb]=TVPowerBalancedFlow_realtime(PNThor,PNRoadNetwork,PNPowerNetwork,PNInitialConditions,PNRebWeight,PNPassengers,PNFlags);
    assert(exitflag>exit_error_flag,'ERROR: power network is infeasible even with load relaxation!')
    
    % Get by how much we had to relax the network
    [GenLoadsTemp,PowerPricesTemp,ChargerPricesTemp,DroppedPowerUP,DroppedPowerDN] = TVPowerBalancedFlow_getpowerdata(cplex_out,lambdas,dual_prices_ix,dual_charger_prices_ix,PNThor,PNRoadNetwork,PNPowerNetwork,PNInitialConditions,PNRebWeight,PNPassengers,PNFlags);
    
    if ~ChargeVoLL
    % Re-solve with the modified loads
        PNPowerNetwork2=PNPowerNetwork;
        PNPowerNetwork2.PowerExtLoads = PNPowerNetwork2.PowerExtLoads - DroppedPowerUP + DroppedPowerDN;
        PNFlags2 = PNFlags;
        PNFlags2.powernetworkloadrelaxflag=0;
        [cplex_out,fval,exitflag,output,lambdas,dual_prices_ix,dual_charger_prices_ix,lb]=TVPowerBalancedFlow_realtime(PNThor,PNRoadNetwork,PNPowerNetwork2,PNInitialConditions,PNRebWeight,PNPassengers,PNFlags2);
        assert(exitflag>exit_error_flag,'ERROR: power network is infeasible even with relaxed loads!')
    end
end
% Save electricity prices and generator loads across TX

% Check that the problem is minimally feasible: generators are >0,
% prices are non-crazy

[GenLoadsTemp,PowerPricesTemp,ChargerPricesTemp,DroppedPowerUP,DroppedPowerDN] = TVPowerBalancedFlow_getpowerdata(cplex_out,lambdas,dual_prices_ix,dual_charger_prices_ix,PNThor,PNRoadNetwork,PNPowerNetwork,PNInitialConditions,PNRebWeight,PNPassengers,PNFlags);

try
    assert(max(PowerPricesTemp)<=PNPowerNetwork.VoLL);
    assert(min(PowerPricesTemp)>=-PNPowerNetwork.VoLL);
    assert(min(GenLoadsTemp)>=0);
    assert(sum(GenLoadsTemp'<PNPowerNetwork.PowerGensMin-PNPowerNetwork.VoLL_threshold)==0)
    assert(sum(GenLoadsTemp'>PNPowerNetwork.PowerGensMax+PNPowerNetwork.VoLL_threshold)==0)
catch
    disp('WEIRD PRICES OR LOADS! Look into it!')
    %keyboard
    PNFlags.solverflag = 'CPLEX';
    [cplex_out,fval,exitflag,output,lambdas,dual_prices_ix,dual_charger_prices_ix,lb]=TVPowerBalancedFlow_realtime(PNThor,PNRoadNetwork,PNPowerNetwork,PNInitialConditions,PNRebWeight,PNPassengers,PNFlags);
    assert(exitflag>-2,'ERROR: power network is infeasible even with load relaxation!')
    [GenLoadsTemp,PowerPricesTemp,ChargerPricesTemp,DroppedPowerUP,DroppedPowerDN] = TVPowerBalancedFlow_getpowerdata(cplex_out,lambdas,dual_prices_ix,dual_charger_prices_ix,PNThor,PNRoadNetwork,PNPowerNetwork,PNInitialConditions,PNRebWeight,PNPassengers,PNFlags);
    try
        assert(max(PowerPricesTemp)<=PNPowerNetwork.VoLL);
        assert(min(PowerPricesTemp)>=-PNPowerNetwork.VoLL);
        assert(min(GenLoadsTemp)>=0);
        assert(sum(GenLoadsTemp'<PNPowerNetwork.PowerGensMin-PNPowerNetwork.VoLL_threshold)==0)
        assert(sum(GenLoadsTemp'>PNPowerNetwork.PowerGensMax+PNPowerNetwork.VoLL_threshold)==0)
    catch
        disp('CPLEX did not help!')
        keyboard
    end
end


[GenLoads,PowerPrices,ChargerPrices,DroppedPowerUP_post,DroppedPowerDN_post] = TVPowerBalancedFlow_getpowerdata(cplex_out,lambdas,dual_prices_ix,dual_charger_prices_ix,PNThor,PNRoadNetwork,PNPowerNetwork,PNInitialConditions,PNRebWeight,PNPassengers,PNFlags);

if PowerNetworkViolation <=0.5 || ~ChargeVoLL
    assert(sum(DroppedPowerUP_post)==0);
    assert(sum(DroppedPowerDN_post)==0);
    DroppedPowerUP = DroppedPowerUP_post;
    DroppedPowerDN = DroppedPowerDN_post;
else
    assert(sum(DroppedPowerUP ~= DroppedPowerUP_post)==0);
    assert(sum(DroppedPowerDN ~= DroppedPowerDN_post)==0);
end
