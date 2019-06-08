function [GenLoads,PowerPrices,ChargerPrices,DroppedPowerUP,DroppedPowerDN] = TVPowerBalancedFlow_getpowerdata(cplex_out,lambdas,dual_prices_ix,dual_charger_prices_ix,Thor,RoadNetwork,PowerNetwork,InitialConditions,RebWeight,Passengers,Flags);

PowerPrices = lambdas.eqlin(dual_prices_ix>0);
ChargerPrices = zeros(size(PowerNetwork.RoadToPowerMap))';
for genid = 1:length(PowerNetwork.RoadToPowerMap)
    ChargerPrices(genid) = PowerPrices(PowerNetwork.RoadToPowerMap(genid));
end

TVPowerBalancedFlowFinder_realtime;

GenLoads=zeros(NumGenerators,1);
for g=1:NumGenerators
    GenLoads(g,1)=cplex_out(FindGeneratorti(Thor,g));
end

t=1;
DroppedPowerUP = zeros(NP,1);
DroppedPowerDN = zeros(NP,1);

if Flags.powernetworkloadrelaxflag
    for i=1:NP
        DroppedPowerUP(i,t) = cplex_out(FindPowerNetworkRelaxUPti(t,i)); % We drop power
        DroppedPowerDN(i,t) = cplex_out(FindPowerNetworkRelaxDNti(t,i)); % We add power
    end
end

%Adjust prices in case there was a blackout
if sum(DroppedPowerUP(i,t))~=0 || sum(DroppedPowerDN(i,t))~=0
    for i=find(DroppedPowerUP>PowerNetwork.VoLL_threshold)
        PowerPrices(i)=PowerNetwork.VoLL;
    end
    for i=find(DroppedPowerDN>PowerNetwork.VoLL_threshold)
        PowerPrices(i)=-PowerNetwork.VoLL;
    end
    for genid = 1:length(PowerNetwork.RoadToPowerMap)
        ChargerPrices(genid) = PowerPrices(PowerNetwork.RoadToPowerMap(genid));
    end
end


