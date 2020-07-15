close all    % Close all open figures
clear        % Reset variables
clc          % Clear the command window

%LTEV2Vsim('help');

% Configuration file
configFile = 'BenchmarkPoisson.cfg';

% Simulation time (s)
T = 10;

% Beacon size (bytes)
B = 300;

%% LTE-V2V Controlled (BRAlgorithm 7)
% Network-controlled allocation Maximum Reuse Distance (MRD)
% Parameters:
% - Interval of scheduled reassignment: Treassign (s)
% 
% LTEV2Vsim(configFile,'simulationTime',T,'BRAlgorithm',7,'Treassign',2,'Raw',150,...
%     'beaconSizeBytes',B);

%% LTE Autonomous (3GPP Mode 4)
% Autonomous allocation algorithm defined in 3GPP standard
density = [150,300, 450, 600, 750];
for i=1:length(density)
LTEV2Vsim('BenchmarkPoisson.cfg','simulationTime',40,'BRAlgorithm',8,'printLOG', false,'ITTpercent', 100,...
    'Raw',475,'rangeForVehicleDensity', 100, 'MCS', 7, 'rho', density(i) ,'smoothingFactorForITT', 1.0,...                   % 100
    'beaconSizeBytes',300, 'powerControl', false, 'rateControl', true, 'Mborder', 0,...
    'printUpdateDelay', true, 'roadLength', 1800, 'maxPtx_dBm',23, 'minPtx_dBm',5);
end

% LTEV2Vsim('BenchmarkPoisson.cfg','simulationTime',50,'BRAlgorithm',8,'printLOG', false,'ITTpercent', 100,...
%     'Raw',475,'rangeForVehicleDensity', 100, 'MCS', 7, 'rho', 750 ,'smoothingFactorForITT', 1.0,...                   % 100
%     'beaconSizeBytes',300, 'powerControl', false, 'rateControl', true, 'Mborder', 0,...
%     'printUpdateDelay', true, 'roadLength', 1800, 'maxPtx_dBm',23, 'minPtx_dBm',5);

