function [RebPaths,PaxChargeLevels]=TVPowerBalancedFlowSampler_realtime(cplex_out,Thor,RoadNetwork,PowerNetwork,InitialConditions,RebWeight,Passengers,Flags,tsample,lb)

%Samples a solution to TVPowerBalancedFlow_realtime and returns rebalancing
%paths and 'hints' on the charge level to use for customers
% Inputs:
% - cplex_out, the first output of TVPowerBalancedFlow_realtime
% - all inputs of TVPowerBalancedFlow_realtime
% - tsample, the time at which the sample is taken. Defaults to one for
% MPC.
% Outputs:
% - RebPaths, a N-by-C cell structure. RP{i,c} is a list of rebalancing
% paths departing from i at charge level c. The list may be empty. The list
% may contain charging and discharging paths.
% - PaxChargeLevels, a N by N cell structure. PCL{i,j} contains a list of
% charge levels for the vehicles that will pick up passengers going from i
% to j.

THRES_ASSERT=1e-2; %Sometimes the optimizer returns "dirty" results. We throw an error if the error is larger than this value;
%Yes, fuck it

% Load finders.
DIAGNOSTIC_FLAG = 0;
TVPowerBalancedFlowFinder_realtime;

tsample=1;
%fprintf('At time t=%d..\n',t)
% Passengers

PaxChargeLevels=cell(N,N);

for k=1:M
    for ssi=1:length(Sources{k})
        if StartTimes{k}(ssi)==tsample
            DepartingPax=Flows{k}(ssi)-cplex_out(FindSourceRelaxks(k,ssi));
            DepProbAtC=zeros(C,1);
            for c=1:C
                DepProbAtC(c)=cplex_out(FindPaxSourceChargecks(c,k,ssi));
                %             if abs(cplex_out(FindPaxSourceChargecks(c,k,ssi)))>=0 && StartTimes{k}(ssi)==t
                %                 fprintf('%f passenger(s) departs from %d to %d at charge level %d\n',cplex_out(FindPaxSourceChargecks(c,k,ssi)),Sources{k}(ssi),Sinks(k),c)
                %             end
            end
            DepProbAtC=DepProbAtC/DepartingPax;

            DepartingPax=round(DepartingPax);
            if DepartingPax==0
                continue
            end
            
            assert(abs(sum(DepProbAtC)-1)<=THRES_ASSERT,'Something wrong with the CPLEX output (sum of departure probabilities for source %d, sink %d is %f (should be 1)\n',ssi,k,sum(DepProbAtC));
            % Two auxiliary vectors that will help us sample
            DepProbAtC_cumsum=cumsum(DepProbAtC);
            DepProbAtC_shiftedcumsum=[0;DepProbAtC_cumsum(1:end-1)];
            
            % Sample departure charge level
            randPax=rand(DepartingPax,1);
            for pax=1:DepartingPax
                sampledCharge=find(DepProbAtC_cumsum>=randPax(pax) & DepProbAtC_shiftedcumsum<randPax(pax));
                assert(length(sampledCharge)==1,'Federico, you messed up the sampling (customers)')
                PaxChargeLevels{Sources{k}(ssi),Sinks(k)}=[PaxChargeLevels{Sources{k}(ssi),Sinks(k)},sampledCharge];
            end
        end
    end
end

% Starter passengers
for k=1:MS
    for ssi=1:length(StarterSources{k})
        DepartingStarters=StarterFlows{k}(ssi)-cplex_out(FindStarterSourceRelaxks(k,ssi));
        DepProbAtTC=zeros(Thor,C);
        %Now, how many do we sample here? It could be less than one!
        % We sample two things: 1. how many people depart at t=1 (over the
        % time distribution of departures) and 2. how many people depart at
        % each charge level.        
        for tt=1:Thor
            for c=1:C
                DepProbAtTC(tt,c)=cplex_out(FindStarterSourceChargetcks(tt,c,k,ssi));
            end
        end
        DepProbAtTC=DepProbAtTC/DepartingStarters; %Normalize
        
        DepartingStarters=round(DepartingStarters);
        if DepartingStarters==0
            continue
        end
        
        assert(abs(sum(sum(DepProbAtTC))-1)<=THRES_ASSERT,'Something wrong with the CPLEX output (sum of departure probabilities for waiting source %d, sink %d is %f (should be 1)\n',ssi,k,sum(sum(DepProbAtTC)));
        % Two auxiliary vectors that will help us sample. We only care
        % about t=1;
        DepProbAtTC_cumsum=cumsum(DepProbAtTC(tsample,:))';
        DepProbAtTC_shiftedcumsum=[0;DepProbAtTC_cumsum(1:end-1)];
        
        % Sample departure charge level
        randPax=rand(DepartingStarters,1);
        for pax=1:DepartingStarters
            sampledCharge=find(DepProbAtTC_cumsum>=randPax(pax) & DepProbAtTC_shiftedcumsum<randPax(pax));
            assert(length(sampledCharge)<=1,'Federico, you messed up the sampling (starters)')
            if ~isempty(sampledCharge)
                PaxChargeLevels{StarterSources{k}(ssi),StarterSinks(k)}=[PaxChargeLevels{StarterSources{k}(ssi),StarterSinks(k)},sampledCharge];
            end
        end
    end
end

RebPaths=cell(N,C);

% Reb. vehicles: travel
for i=1:N
    llist=find(ChargersList==i); %Is this node a charging node?
    
    for c=1:C
        NumRebVehiclesAtC=0;
        DepProbAtCi=zeros(N,1);
        for j=RoadGraph{i}
            if (abs(cplex_out(FindRoadLinkRtcij(tsample,c,i,j)))>=0)
                NumRebVehiclesAtC=NumRebVehiclesAtC+cplex_out(FindRoadLinkRtcij(tsample,c,i,j));
                DepProbAtCi(j)=cplex_out(FindRoadLinkRtcij(tsample,c,i,j));
%                 if j~=i
%                     fprintf('%f reb. vehicle(s) drive from %d to %d at charge level %d\n',cplex_out(FindRoadLinkRtcij(t,c,i,j)),i,j,c)
%                 else
%                     %fprintf('%f reb. vehicle(s) stay at %d at charge level %d\n',cplex_out(FindRoadLinkRtcij(t,c,i,j)),i,c)
%                 end
            end
        end
        
        if ~isempty(llist)
            lnum=length(llist);
            lidx=1;
            for l=llist
                NumRebVehiclesAtC=NumRebVehiclesAtC+cplex_out(FindChargeLinkRtcl(tsample,c,l));
                NumRebVehiclesAtC=NumRebVehiclesAtC+cplex_out(FindDischargeLinkRtcl(tsample,c,l));
                DepProbAtCi(N+2*lidx-1)=cplex_out(FindChargeLinkRtcl(tsample,c,l));
                DepProbAtCi(N+2*lidx)=cplex_out(FindDischargeLinkRtcl(tsample,c,l));
                lidx=lidx+1;
            end
        end
        DepProbAtCi=DepProbAtCi/NumRebVehiclesAtC;
        NumRebVehiclesAtC=round(NumRebVehiclesAtC);
        if NumRebVehiclesAtC==0
            continue
        end

        assert(abs(sum(DepProbAtCi)-1)<=THRES_ASSERT,'Federico, you messed up the rebalancing cumsums')
        DepProbAtCi_cumsum=cumsum(DepProbAtCi);
        DepProbAtCi_shiftedcumsum=[0;DepProbAtCi_cumsum(1:end-1)];
        randReb=rand(NumRebVehiclesAtC,1);
        for rebix=1:NumRebVehiclesAtC
            sampledCharge=find(DepProbAtCi_cumsum>=randReb(rebix) & DepProbAtCi_shiftedcumsum<randReb(rebix));

            assert(length(sampledCharge)==1,'Federico, you messed up the sampling (rebalancers)')

            if sampledCharge<=N
                if isempty(RebPaths{i,c})
                    RebPaths{i,c}={[i, sampledCharge]};
                else
                    RebPaths{i,c}=[RebPaths{i,c},[i, sampledCharge]];
                end
            elseif mod(sampledCharge-N,2)==1  % Odd entry: charging
                if isempty(RebPaths{i,c})
                    RebPaths{i,c}={[i, -N-i]};
                else
                    RebPaths{i,c}=[RebPaths{i,c},[i, -N-i]];
                end
            else %Even entry: discharging
                if isempty(RebPaths{i,c})
                    RebPaths{i,c}={[i, -i]};
                else
                    RebPaths{i,c}=[RebPaths{i,c},[i, -i]]; %Also, this in-band signaling is a HACK.
                end
            end
        end
        
    end
end
