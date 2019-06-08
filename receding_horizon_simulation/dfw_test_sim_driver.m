clear all; close all; clc;


addpath(genpath('../solvers')); %Hack to get the functions in the root folder. Will remove.

addpath(genpath('utilities'));
addpath(genpath('rebalancers'));

% LP solvers
addpath(genpath('~/mosek/8/toolbox/r2014a'));
addpath(genpath('/opt/ibm/ILOG/CPLEX_Studio128/cplex/matlab/x86-64_linux'))


NUM_SIMS=12;

ControllerModes = {'PAMoD', 'GREEDY','POWER'};

fileName.dataName = '../case_studies/Dallas_Fort-Worth/scenario/dfw_roadgraph_tv_25cl_19h_900step_returntripsTrue_operational_2016cost.mat';
fileName.busLocName = '../case_studies/Dallas_Fort-Worth/scenario/dfw_buslocations_25cl_windsun_v3';
fileName.saveNameSuffix = '_realtime';
fileName.saveFolder = 'results/';

load(fileName.dataName);

timeSettings.timeSteps = 1;              % number of time steps per minute
timeSteps = timeSettings.timeSteps;
timeSettings.dt = 60/timeSteps;         % length of each time step (in seconds)
timeSettings.Tmax = 600*timeSteps;      % simulation time, in steps.
timeSettings.Treb = 5*timeSteps;
timeSettings.Thor_reb = 4*60*60; % horizon of the MPC optimizer, here 2 hours, in SECONDS
timeSettings.predictionTimeStep=15*60;  % time step of the MPC optimizer, here 30 minutes, in SECONDS
timeSettings.Tmincharge = 5*timeSteps; % minimum duration of a charging-discharging action
timeSettings.Tthresh = 0;     % at what time to start gathering data


controllerSettings.v = 4.5*1e5;
controllerSettings.NAIVEFLAG = 0;
controllerSettings.UNPLANNEDPAXFLAG = 1; % If set, whenever a precomputed pax route is not available, calls A*. If unset, waits until next rebalancing horizon.
controllerSettings.forecastPeriods = 10;
controllerSettings.initialCarCharge = round(C/2);
controllerSettings.MinEndCharge = round(C/2);
controllerSettings.drop_dead_charge=-2; %The car will stop moving if the charge drops below this level
controllerSettings.findRouteMode = 2; %1: use A*. 2: use cached routes
controllerSettings.VideoLog = 0;
controllerSettings.customerStdDev = .33; % 10% perturbation on number of customers
controllerSettings.powerStdDev = .016; % 5% perturbation on demand for power


for i = 1:NUM_SIMS
    for cmdix = 1:length(ControllerModes)
        controllerMode = ControllerModes{cmdix};
        dfw_test_sim(controllerMode,fileName,timeSettings,controllerSettings)
    end
end
