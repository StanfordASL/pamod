function [Passengers] = predictPAMoDDemand(currentTime, horizon,predictionTimeStep,RebOptions)

% Inputs: current time (in simulation steps), horizon (in simulation steps)
% Output: Sinks, Sources, StartTimes, FlowsIn, in the format specified in
% TVPowerBalancedFlow_realtime.

load(RebOptions.predictFileName);
assert(timeStepSize==predictionTimeStep,'ERROR: the time step size hardcoded in predictPAMoDDemand does not match the time step in the optimizer')

TrimmedStartTimes=StartTimes(StartTimes>=currentTime & StartTimes<=currentTime+horizon);
TrimmedSinks=Sinks(StartTimes>=currentTime & StartTimes<=currentTime+horizon);
TrimmedSources=Sources(StartTimes>=currentTime & StartTimes<=currentTime+horizon);
TrimmedFlows=FlowsIn(StartTimes>=currentTime & StartTimes<=currentTime+horizon);

CompactSinks=unique(TrimmedSinks);
CompactSinks=CompactSinks'; %We like column vectors

CompactSources=cell(length(CompactSinks),1);
CompactStartTimes=cell(length(CompactSinks),1);
CompactFlows=cell(length(CompactSinks),1);

for i=1:length(CompactSinks)
    CompactSources{i}=TrimmedSources(TrimmedSinks==CompactSinks(i));
    CompactStartTimes{i}=TrimmedStartTimes(TrimmedSinks==CompactSinks(i));
    CompactFlows{i}=TrimmedFlows(TrimmedSinks==CompactSinks(i));
end

if length(CompactSinks) == 0
    CompactSinks = [1];
    CompactSources = {[1]};
    CompactStartTimes = {[1]};
    CompactFlows = {[0]};
end

Passengers.Sinks=CompactSinks;
Passengers.Sources=CompactSources;
Passengers.StartTimes=CompactStartTimes;
Passengers.Flows=CompactFlows;