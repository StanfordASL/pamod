function [PowerStats] = ExtractSimPowerStats(settings,Stats,PowerNetwork,isbusinDallas)

% Stats.GenLoadsLog   %Generator load at time t
% Stats.ChargersLoadLog %TSO charger load at time t
% Stats.PowerPricesLog %Power prices (all) at time t
% Stats.ChargerPricesLog %Power prices (chargers) load at time t

numGenerators=size(PowerNetwork.PowerCosts,2);
numBuses = size(PowerNetwork.PowerExtLoads,1);

Gen_costs = zeros(numGenerators,settings.Tmax);
Ext_loads = zeros(numBuses,settings.Tmax);
All_loads = zeros(numBuses,settings.Tmax);
for t=1:settings.Tmax
    mytime = max(round(t*settings.dt/settings.predictionTimeStep),1);
    Gen_costs(:,t) = PowerNetwork.PowerCosts(mytime,:)'.*3600/settings.predictionTimeStep; %convert to price per hour
    Ext_loads(:,t) = PowerNetwork.PowerExtLoads(:,mytime);
    All_loads(:,t) = Ext_loads(:,t);
    for g=1:length(PowerNetwork.RoadToPowerMap)
        All_loads(PowerNetwork.RoadToPowerMap(g),t) = All_loads(PowerNetwork.RoadToPowerMap(g),t)+Stats.ChargersLoadLog(g,t);
    end
end

Energy_demand_log = sum(Ext_loads)+sum(Stats.ChargersLoadLog);
Energy_demand_tot = sum(Energy_demand_log)*settings.dt/3600;

Gen_cost_log = sum((Stats.GenLoadsLog.*Gen_costs).*settings.dt/3600);
Gen_cost_tot = sum(Gen_cost_log);

Ext_expense_log = sum((Ext_loads.*Stats.PowerPricesLog).*settings.dt/3600);
Ext_expense_tot = sum(Ext_expense_log);

DFW_expense_log = sum((Ext_loads(isbusinDallas,:).*Stats.PowerPricesLog(isbusinDallas,:)).*settings.dt/3600);
DFW_expense_tot = sum(DFW_expense_log);

DFW_unit_expense_log = sum((Ext_loads(isbusinDallas,:).*Stats.PowerPricesLog(isbusinDallas,:)))./sum(Ext_loads(isbusinDallas,:));
%DFW_unit_expense_tot = mean(DFW_unit_expense_log(1:end-1));
DFW_unit_expense_tot = DFW_expense_tot/(sum(sum(Ext_loads(isbusinDallas,:))).*settings.dt/3600); %average in loads, not in time.


TSO_expense_log = sum((Stats.ChargersLoadLog.*Stats.ChargerPricesLog).*settings.dt/3600);
TSO_expense_tot = sum(TSO_expense_log);


PowerStats.Energy_demand_tot=Energy_demand_tot;
PowerStats.Energy_demand_log=Energy_demand_log;
PowerStats.Ext_loads_log=Ext_loads;
PowerStats.All_loads_log=All_loads;
PowerStats.Gen_cost_log=Gen_cost_log;
PowerStats.Gen_cost_tot=Gen_cost_tot;
PowerStats.Ext_expense_log=Ext_expense_log;
PowerStats.Ext_expense_tot=Ext_expense_tot;
PowerStats.TSO_expense_log=TSO_expense_log;
PowerStats.TSO_expense_tot=TSO_expense_tot;
PowerStats.DFW_expense_log=DFW_expense_log;
PowerStats.DFW_expense_tot=DFW_expense_tot;
PowerStats.DFW_unit_expense_tot=DFW_unit_expense_tot;

fprintf('Case: %s\nTot. energy demand %d\nGeneration cost (all): %d\nNon-TSO expense: %d\nTSO expense: %d\nDFW expense: %d (per 100MW: %d)\nOverall expense: %d\n',...
    settings.ControllerMode,Energy_demand_tot,Gen_cost_tot,Ext_expense_tot,TSO_expense_tot,DFW_expense_tot,DFW_unit_expense_tot,Ext_expense_tot+TSO_expense_tot)

fprintf('Easy-to-copy: \n %d \t %d \t %d \t %d \t %d \t %d \t %d \t\n',...
    Energy_demand_tot,Gen_cost_tot,Ext_expense_tot,TSO_expense_tot,DFW_expense_tot,DFW_unit_expense_tot,Ext_expense_tot+TSO_expense_tot)
