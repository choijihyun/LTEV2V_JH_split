function [simValues,outputValues,appParams,simParams,phyParams,outParams] = mainLTEV2V(appParams,simParams,phyParams,outParams,simValues,outputValues)
% The function mainLTEV2V() performs the LTE-V2V simulation

%% First Radio Resources Assignment

% Initialize Tx power per RB vector
phyParams.maxPtxERP_RB = phyParams.PtxERP_RB; % Max PtxERP
phyParams.PtxERP_RB = phyParams.PtxERP_RB*ones(simValues.maxID,1);

%%%%%%%%%%%%
elapsedTime = 0;

% ITT for each vehicles
ITT = appParams.Tbeacon * ones(simValues.maxID,1);

% phyParams.maxPtx_dBm_RB = phyParams.Ptx_dBm_RB; % Max PtxERP
phyParams.Ptx_dBm_RB = phyParams.Ptx_dBm_RB*ones(simValues.maxID,1);
%%%%%%%%%5

% Number of groups for position update
NPosUpdate = round(simParams.Tupdate/appParams.Tbeacon);

% Assign update period to all vehicles (introduce a position update delay)
posUpdateAllVehicles = randi(NPosUpdate,simValues.maxID,1);

% Copy real coordinates of vehicles
simValues.Xvehicle = simValues.XvehicleReal;
simValues.Yvehicle = simValues.YvehicleReal;

%%%%%%%%%%%%%%%%%
simValues.RXpowerSum = zeros(simValues.maxID,appParams.Nbeacons);
simValues.RXpowerHist = zeros(simValues.maxID, appParams.Nbeacons * 11);

CBPThresh_dBm = simParams.CBRThresh; % -107 (dBm)
CBPThresh = 10^((CBPThresh_dBm-30)/10); % -107 dBm, 1.9953e-14
SensingPwr_dBm = simParams.sensingPwr;
SensingPwr = 10^((SensingPwr_dBm-30)/10);
simValues.PowerThreshHistory = zeros(simValues.maxID, 6); % max, min, last, select pool num
% simValues.CBPMatrix = zeros(simValues.maxID,ceil(simParams.simulationTime/simParams.simulationStep));
simValues.CBPMatrix = zeros(simValues.maxID,ceil(simParams.simulationTime/appParams.Tbeacon));
CBPMatrixHist = zeros(simValues.maxID,ceil(simParams.simulationTime/appParams.Tbeacon));

% simValues.CRHist = zeros(simValues.maxID, 10*(5+simParams.simulationTime) * (1+simParams.reTxTrialMax) );
simValues.CRHistIdx = ones(simValues.maxID, 1);
simValues.CR = zeros(simValues.maxID,ceil(simParams.simulationTime/simParams.simulationStep));
phyParams.Ptx_dBm_RB_Hist = zeros(simValues.maxID,ceil(simParams.simulationTime/appParams.Tbeacon));
%%%%%%%%%%%%%%%%5


% Find vehicles not at the borders
indexNoBorder = find((simValues.XvehicleReal(:,1)>=(simValues.Xmin+simParams.Mborder) & simValues.XvehicleReal(:,1)<=(simValues.Xmax-simParams.Mborder)) & (simValues.YvehicleReal(:,1)>=(simValues.Ymin+simParams.Mborder) & simValues.YvehicleReal(:,1)<=(simValues.Ymax-simParams.Mborder)));

% Call function to compute distances
% Distance matrix has dimensions equal to IDvehicle x IDvehicle in order to
% speed up the computation (only vehicles present at the considered instant)
% distance(i,j): distance from vehicle with index i to vehicle with index j
[distance,~,~,~,allNeighborsID] = computeDistance(simValues.Xvehicle,simValues.Yvehicle,simValues.IDvehicle,phyParams.Raw,phyParams.RawMax);

% Save distance matrix
distanceRealOld = distance;

% Initialize array of old of coordinates
XvehicleRealOld = simValues.XvehicleReal;
YvehicleRealOld =  simValues.YvehicleReal;

% Initialize array of old angles
angleOld = zeros(length(XvehicleRealOld),1);

% Initialize BRid
% BRid = -2*ones(simValues.maxID,1);
%%%%%%%%%%%%
%BRid : BRid which vehicle used
%BRpassCounter : in rate control, number of snap to pass
%BRidAfterPass : in pass snap, maintain BRid because BRid ==-2(BRidAfterPass is will be used at future)
BRid = -2*ones(simValues.maxID,1);
BRidFake = -1*ones(simValues.maxID,1);
BRidAfterPass = -2*ones(simValues.maxID,1);

BRidBackup = -2*ones(simValues.maxID,1);
BRpassCounter = ones(simValues.maxID,1);
TransmitFlag = ones(simValues.maxID,1);

totalUse = zeros(200, 1);

%jihyun
% rng;
% BRpassRandom = randi( 3, simValues.maxID, 1);
% BRpassCounter = BRpassRandom;
% initialVehicle = find(BRpassRandom~=1);

% sumOfUpdateDelayForDistance = zeros(simParams.roadLength/10,1);
% numOfUpdateForDistance = ones(simParams.roadLength/10,1);
%IA
sumOfUpdateDelayForDistance = zeros(60,1);
numOfUpdateForDistance = ones(60,1);

%for rate control
RatesumOfUpdateDelayForDistance = zeros(60,1);
RatenumOfUpdateForDistance = ones(60,1);
tmpRatesumOfUpdateDelayForDistance = zeros(60,1);
tmpRatenumOfUpdateForDistance = ones(60,1);

%for power control
PowersumOfUpdateDelayForDistance = zeros(60,1);
PowernumOfUpdateForDistance = ones(60,1);
tmpPowersumOfUpdateDelayForDistance = zeros(60,1);
tmpPowernumOfUpdateForDistance = ones(60,1);

%PDR
numOfTx = ones(simParams.roadLength/10,1);
numOfRx = ones(simParams.roadLength/10,1);

%for rate control
RatenumOfTx = ones(simParams.roadLength/10,1);
RatenumOfRx = ones(simParams.roadLength/10,1);
tmpRatenumOfTx = ones(simParams.roadLength/10,1);
tmpRatenumOfRx = ones(simParams.roadLength/10,1);

%for power control
PowernumOfTx = ones(simParams.roadLength/10,1);
PowernumOfRx = ones(simParams.roadLength/10,1);
tmpPowernumOfTx = ones(simParams.roadLength/10,1);
tmpPowernumOfRx = ones(simParams.roadLength/10,1);

NerrorHist = -2*ones(1000,1);
BLERHist = -2*ones(1000 * 605,2);
windowForUniqueVehicle = zeros(simValues.maxID, simValues.maxID-1, 10);
%slopeDefault = 0.5 / (25 * 6 - 25);
% change slopeDefault based on ITT on VD to CBP

%% ITT watermark & default slope- jihyun
% ITT_y = 0.1;
% ITT_x = 0.03;
ITT_y = 0.3;
ITT_x = 0.1;
slopeDefault = (ITT_y - ITT_x) / (0.8 - 0.5);
%%%%%%%%%%%5

%% number of power control and rate control - jihyun
% ratio of control 
powerRatio = appParams.ratio * 0.1;
rateRatio = 1-powerRatio;

% each number of power control and rate control
powerVehicle = round(simValues.maxID*powerRatio);
rateVehicle = round(simValues.maxID*rateRatio);


%% BRreassignment by BRAlgorithm
if simParams.BRAlgorithm==101
    % Call Benchmark Algorithm 101 (RANDOM ALLOCATION)
    [BRid] = BRreassignmentRandom(simValues.IDvehicle,BRid,appParams.Nbeacons,indexNoBorder);
    
elseif simParams.BRAlgorithm==102
    % Call Benchmark Algorithm 102 (ORDERED ALLOCATION)
    [BRid] = BRreassignmentOrdered(simValues.XvehicleReal,simValues.IDvehicle,BRid,appParams.Nbeacons,indexNoBorder);
    
elseif simParams.BRAlgorithm==7
    % Call first assignment of BR Algorithm 7 (CONTROLLED with MAXIMUM REUSE)
    scheduledID = simValues.IDvehicle;
    [BRid] = BRreassignmentControlledMaxReuse(simValues.IDvehicle,BRid,scheduledID,allNeighborsID,appParams.NbeaconsT,appParams.NbeaconsF,indexNoBorder);
else
    % First BRs assignment (Always CONTROLLED)
    % BRid -> BR assigned
    %   -1 -> blocked
    %   -2 -> vehicle outside the scenario (unknown position)
    [BRid] = firstAssignment(simValues.IDvehicle,BRid,distance,appParams.Nbeacons,indexNoBorder,phyParams.Rreuse,simParams.randomOrder);
    %jihyun - BRid랑 BRidFake를 동일하게 유지하기 위하여
    BRidFake = BRid;
    % for complete BRid which is not impact from other things
%     BRidBackup = BRid;
%     %%% this for random transmit in rate control
%     if simParams.rateControl && (~simParams.powerControl)
%         BRidAfterPass(initialVehicle) = BRid(initialVehicle);
%         BRidFake(initialVehicle) = BRid(initialVehicle);
%         BRid(initialVehicle) = -2;
%     end
end

if simParams.BRAlgorithm==5
    % Initialization of oldBRid for the first cycle
    oldBRid = BRid;
end

if simParams.BRAlgorithm==2 || simParams.BRAlgorithm==6 || simParams.BRAlgorithm==7
    % Number of groups for scheduled resource reassignment (BRAlgorithm=2 or 6)
    NScheduledReassign = round(simParams.Treassign/appParams.Tbeacon);
    
    % Assign update period to vehicles (BRAlgorithm=2 or 6)
    scheduledReassign = randi(NScheduledReassign,simValues.maxID,1);
end

if simParams.BRAlgorithm==8
    % Find min and max values for random counter (BRAlgorithm=8)
    [simParams.minRandValueMode4,simParams.maxRandValueMode4] = findRandValueMode4(appParams.Tbeacon,simParams);
    
    % Initialize reselection counter (BRAlgorithm=8)
    resReselectionCounter = Inf*ones(simValues.maxID,1);
    % Generate values for vehicles in the scenario (first cycle)
    resReselectionCounter(simValues.IDvehicle) = (simParams.minRandValueMode4-1) + randi((simParams.maxRandValueMode4-simParams.minRandValueMode4)+1,1,length(simValues.IDvehicle));
    % Set value 0 to vehicles that are blocked
    resReselectionCounter(BRid==-1)=0;
    
    % Initialization of sensing matrix (BRAlgorithm=8)
%     sensingMatrix = zeros(simParams.NsensingPeriod,appParams.Nbeacons,simValues.maxID);
%jihyun - for memory 11 
    sensingMatrix = zeros(simParams.NsensingPeriod+1,appParams.Nbeacons,simValues.maxID);
    knownUsedMatrix = zeros(appParams.Nbeacons,simValues.maxID);
end

% Initialization of lambda: SINR threshold for BRAlgorithm 9
if simParams.BRAlgorithm==9
    lambda = phyParams.gammaMin;
end

% Initialization of packet generation time
timeNextPacket = appParams.Tbeacon * rand(simValues.maxID,1);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Simulation Cycle

% Number of snapshots
simValues.snapshots = ceil(simParams.simulationTime/appParams.Tbeacon);

% Start stopwatch
tic

fprintf('Simulation Time: ');
reverseStr = '';

for snap = 1:simValues.snapshots
    
    % Print time to video
    elapsedTime = round(snap*appParams.Tbeacon*100)/100;
    if snap==1
        % Print first cycle without estimation of end
        msg = sprintf('%.1f / %.1fs',elapsedTime,simParams.simulationTime);
        fprintf([reverseStr, msg]);
        reverseStr = repmat(sprintf('\b'), 1, length(msg));
    else
        reverseStr = printUpdateToVideo(elapsedTime,simParams.simulationTime,reverseStr);
    end
    
    %% Position Update
    
    if mod(elapsedTime,simValues.timeResolution)==0 || snap==1
        % Update position only if timeElapsed is a multiple of timeResolution
        % or if it is the first cycle of simulation
        
        if ~simParams.fileTrace % 0
            % Call function to update vehicles real positions
            [simValues.XvehicleReal,simValues.YvehicleReal,indexNewVehicles,indexOldVehicles,indexOldVehiclesToOld,IDvehicleExit] = updatePosition(simValues.XvehicleReal,simValues.YvehicleReal,simValues.IDvehicle,simValues.v,simValues.direction,appParams.Tbeacon,simValues.Xmax);
        else
            % If it is the first cycle of the simulation, keep the
            % positions at time zero, else read positions at elapsed time
            if snap==1 && simValues.timeResolution~=appParams.Tbeacon
                updateTime = 0;
            else
                updateTime = elapsedTime;
            end
            
            % Store IDs of vehicles at the previous beacon period and update positions
            [simValues.XvehicleReal,simValues.YvehicleReal,simValues.IDvehicle,indexNewVehicles,indexOldVehicles,indexOldVehiclesToOld,IDvehicleExit] = updatePositionFile(updateTime,simValues.dataTrace,simValues.IDvehicle);
            % Update BRid vector (variable number of vehicles in the scenario)
            [BRid] = updateBRidFile(BRid,simValues.IDvehicle,indexNewVehicles);
        end
        
        if simParams.BRAlgorithm==8
            % Update resReselectionCounter for vehicles exiting and entering the scenario (BRAlgorithm=8)
            % (vehicles that enter or are blocked start with a counter set to 0)
            resReselectionCounter(IDvehicleExit) = Inf;
            resReselectionCounter(BRid==-1) = 0;
        end
        
        if simParams.BRAlgorithm==3 || simParams.BRAlgorithm==4 || simParams.BRAlgorithm==5
            % No positioning error (AUTONOMOUS)
            simValues.Xvehicle = simValues.XvehicleReal;
            simValues.Yvehicle = simValues.YvehicleReal;
        else
            % Current position update period
            PosUpdatePeriod = mod(round(elapsedTime/appParams.Tbeacon)-1,NPosUpdate)+1;
            
            % Add LTE positioning delay (if selected)
            [simValues.Xvehicle,simValues.Yvehicle,PosUpdateIndex] = addPosDelay(simValues.Xvehicle,simValues.Yvehicle,simValues.XvehicleReal,simValues.YvehicleReal,simValues.IDvehicle,indexNewVehicles,indexOldVehicles,indexOldVehiclesToOld,posUpdateAllVehicles,PosUpdatePeriod);
            
            % Add LTE positioning error (if selected)
            % (Xvehicle, Yvehicle): fictitious vehicles' position seen by the eNB
            [simValues.Xvehicle(PosUpdateIndex),simValues.Yvehicle(PosUpdateIndex)] = addPosError(simValues.XvehicleReal(PosUpdateIndex),simValues.YvehicleReal(PosUpdateIndex),simParams.sigmaPosError);
        end
        
        % Find vehicles not at the borders
        indexNoBorder = find((simValues.XvehicleReal(:,1)>=(simValues.Xmin+simParams.Mborder) & simValues.XvehicleReal(:,1)<=(simValues.Xmax-simParams.Mborder)) & (simValues.YvehicleReal(:,1)>=(simValues.Ymin+simParams.Mborder) & simValues.YvehicleReal(:,1)<=(simValues.Ymax-simParams.Mborder)));
        
        % Call function to compute distances
        % distance(i,j): distance from vehicle with index i to vehicle with index j
        [distanceReal,awarenessID,neighborsDistance,neighborsID,allNeighborsID] = computeDistance(simValues.XvehicleReal,simValues.YvehicleReal,simValues.IDvehicle,phyParams.Raw,phyParams.RawMax);
        if ~simParams.posError95 && NPosUpdate==1
            distance = distanceReal;
        else
            [distance,~,~,~,allNeighborsID] = computeDistance(simValues.Xvehicle,simValues.Yvehicle,simValues.IDvehicle,phyParams.Raw,phyParams.RawMax);
        end
        
        % Call function to update distance matrix where D(i,j) is the
        % change in distance of link i to j from time n-1 to time n and used
        % for updating Shadowing matrix
        [dUpdate,simValues.Shadowing_dB,distanceRealOld] = updateDistance(distanceReal,distanceRealOld,indexOldVehicles,indexOldVehiclesToOld,simValues.Shadowing_dB,phyParams.stdDevShadowLOS_dB);
        
        % Call function to compute received power
        % RXpower has dimensions IDvehicle X IDvehicle to speed up computation
        % RXpower(i,j): power received by vehicle with index i from vehicle with index j
        if ~phyParams.winnerModel
            [RXpower,CHgain,simValues.Shadowing_dB,simValues.Xmap,simValues.Ymap] = computeRXpower(simValues.IDvehicle,distanceReal,phyParams.PtxERP_RB,phyParams.Gr,phyParams.L0,phyParams.beta,simValues.XvehicleReal,simValues.YvehicleReal,phyParams.Abuild,phyParams.Awall,simValues.XminMap,simValues.YmaxMap,simValues.StepMap,simValues.GridMap,simParams.fileObstaclesMap,simValues.Shadowing_dB,dUpdate,phyParams.stdDevShadowLOS_dB,phyParams.stdDevShadowNLOS_dB);
        else
            [RXpower,CHgain,simValues.Shadowing_dB,simValues.Xmap,simValues.Ymap] = computeRXpowerWinner(simValues.IDvehicle,distanceReal,phyParams.PtxERP_RB,phyParams.Gr,simValues.XvehicleReal,simValues.YvehicleReal,simValues.XminMap,simValues.YmaxMap,simValues.StepMap,simValues.GridMap,simParams.fileObstaclesMap,simValues.Shadowing_dB,dUpdate,phyParams.stdDevShadowLOS_dB,phyParams.stdDevShadowNLOS_dB, snap, appParams.RBsBeacon);
        end
        
        % Floor coordinates for PRRmap creation (if enabled)
        if outParams.printPRRmap
            simValues.XmapFloor = floor(simValues.Xmap);
            simValues.YmapFloor = floor(simValues.Ymap);
        end
    end
    
    % Call function to calculate effective neighbors (if enabled)
    if simParams.neighborsSelection
        [effAwarenessID,effNeighborsID,XvehicleRealOld,YvehicleRealOld,angleOld] = computeSignificantNeighbors(simValues.IDvehicle,simValues.XvehicleReal,simValues.YvehicleReal,XvehicleRealOld,YvehicleRealOld,neighborsID,indexNewVehicles,indexOldVehicles,indexOldVehiclesToOld,angleOld,simParams.Mvicinity,phyParams.Raw,phyParams.RawMax,neighborsDistance);
    end
    
    %% Error Detection
    
    if ~outParams.printDistanceDetails && simParams.BRAlgorithm~=8
        
        % Create awareness BRid Matrix
        % Row index -> index of the vehicle
        % Column = BRIDs assigned to vehicles in the awareness range
        awarenessBRid = BRidmatrix(awarenessID,BRid);
        
        % Compute SINR of received beacons
        awarenessSINR = computeSINR(simValues.IDvehicle,BRid,appParams.NbeaconsT,appParams.NbeaconsF,awarenessID,RXpower,phyParams.IBEmatrix,phyParams.Ksi,phyParams.PtxERP_RB,phyParams.PnRB);
        
        if simParams.neighborsSelection
            % Keep only significant neighbors (within Raw)
            awarenessID = effAwarenessID;
            awarenessBRid = (awarenessID>0).*awarenessBRid;
        end
        
        % Error detection (within Raw)
        % Create Error Matrix = [ID RX, ID TX, BRid, distance]
        errorMatrix = findErrors(simValues.IDvehicle,awarenessID,awarenessSINR,awarenessBRid,distanceReal,phyParams.gammaMin);
        errorMatrixNoBorder = errorRemoveBorder(simValues.IDvehicle,errorMatrix,indexNoBorder);
        
    else
        
        % Create neighbors and awareness BRid Matrix
        neighborsBRid = BRidmatrix(neighborsID,BRid);
        awarenessBRid = (awarenessID>0).*neighborsBRid;
        
        % Compute SINR of received beacons
        neighborsSINR = computeSINR(simValues.IDvehicle,BRid,appParams.NbeaconsT,appParams.NbeaconsF,neighborsID,RXpower,phyParams.IBEmatrix,phyParams.Ksi,phyParams.PtxERP_RB,phyParams.PnRB);
        
        if simParams.neighborsSelection
            % Keep only significant neighbors (up to RawMax)
            neighborsID = effNeighborsID;
            neighborsBRid = (neighborsID>0).*neighborsBRid;
            
            % Keep only significant neighbors (within Raw)
            awarenessID = effAwarenessID;
            awarenessBRid = (awarenessID>0).*awarenessBRid;
        end
        
        % Error detection (up to RawMax)
        %         errorMatrixRawMax = findErrors(simValues.IDvehicle,neighborsID,neighborsSINR,neighborsBRid,distanceReal,phyParams.gammaMin);
        [errorMatrixRawMax, NerrorHist, BLERHist] = findErrors(simValues.IDvehicle,neighborsID,neighborsSINR,neighborsBRid,distanceReal,phyParams.gammaMin, elapsedTime, NerrorHist, BLERHist);
        errorMatrixRawMaxNoBorder = errorRemoveBorder(simValues.IDvehicle,errorMatrixRawMax,indexNoBorder);
        
        
        % Error detection (within Raw)
        errorMatrix = errorMatrixRawMax(errorMatrixRawMax(:,4)<phyParams.Raw,:);
        errorMatrixNoBorder = errorMatrixRawMaxNoBorder(errorMatrixRawMaxNoBorder(:,4)<phyParams.Raw,:);
        
    end
    
    % Check the correctness of SCI messages
    if simParams.BRAlgorithm==8
        % phyParams.minSCIsinr == 1;
        errorSCImatrix = neighborsSINR < phyParams.minSCIsinr;
    end
    %% power control
    % calculateCBP
    [simValues.RXpowerSum, simValues.CBPMatrix, simValues.RXpowerHist, CBPMatrixHist, TransmitFlag, nonUsed, totalUse] = calculateCBP(CBPMatrixHist, simValues, simValues.CBPMatrix, ...
        simValues.IDvehicle, appParams.NbeaconsT, RXpower, distance, ...
        neighborsID, allNeighborsID, BRid, snap, simValues.RXpowerSum, CBPThresh, appParams,...
        simValues.RXpowerHist,...
        appParams.Nbeacons, appParams.NbeaconsF, simParams.CBRRange,...
        timeNextPacket, elapsedTime, phyParams.Raw, phyParams.rangeForVehicleDensity, simParams.rho, phyParams.MCS, phyParams.smoothingFactorForITT, simParams.ITTpercent, ...
        simParams.printLOG, BRidFake, phyParams.IBEmatrix, phyParams.PnRB, TransmitFlag, distanceReal, totalUse, powerVehicle, rateVehicle, appParams.ratio);
    if simParams.powerControl
        % calculate only power vehicle's TxPower
        phyParams = computeTransmissionPowerforNext(simValues, phyParams, simValues.CBPMatrix(:, snap), snap, simParams.rho, phyParams.smoothingFactorForITT, simParams.ITTpercent, simParams.printLOG, powerVehicle, rateVehicle, appParams.ratio);
        
        %for calculate only for power control vehicle
        index = find(simValues.IDvehicle <= powerVehicle);
        phyParams.Ptx_dBm_RB_Hist(index, snap) = phyParams.Ptx_dBm_RB(index);
        phyParams.Ptx_RB(index, : ) =  10.^((phyParams.Ptx_dBm_RB(index)-30)/10);
        % Compute radiated power per RB - 두 파일의 초기화 과정을 따라서 per RB 도출
        % initiatePhyParameters.m
        phyParams.PtxERP_RB_dBm(index, : ) = phyParams.Ptx_dBm_RB(index) + phyParams.Gt_dB;
        phyParams.PtxERP_RB(index) = 10.^((phyParams.PtxERP_RB_dBm(index)-30)/10);
        % deriveBeaconResources.m
        phyParams.PtxERP_RB(index) = phyParams.PtxERP_RB(index)/(appParams.RBsBeacon/2);
        phyParams.PtxERP_RB_dBm(index) = 10*log10(phyParams.PtxERP_RB(index))+30;
        
%         phyParams.Ptx_dBm_RB_Hist(:, snap) = phyParams.Ptx_dBm_RB;
%         phyParams.Ptx_RB =  10.^((phyParams.Ptx_dBm_RB-30)/10);
%         % Compute radiated power per RB -두 파일의 초기화 과정을 따라서 per RB 도출
%         % initiatePhyParameters.m
%         % effective radiated power(dBm)
%         phyParams.PtxERP_RB_dBm = phyParams.Ptx_dBm_RB + phyParams.Gt_dB;
%         phyParams.PtxERP_RB = 10.^((phyParams.PtxERP_RB_dBm-30)/10);
%         
%         % deriveBeaconResources.m - Rxpower calculate with PtxERP_RB
%         phyParams.PtxERP_RB = phyParams.PtxERP_RB/(appParams.RBsBeacon/2);
%         phyParams.PtxERP_RB_dBm = 10*log10(phyParams.PtxERP_RB)+30;
        
    end
    
    
    %% KPIs Computation (Snapshot)
    
    % Number of vehicles in the scenario (removing border effect)
    outputValues.NvehiclesNoBorder = length(indexNoBorder);
    outputValues.NvehiclesNoBorderTOT = outputValues.NvehiclesNoBorderTOT + outputValues.NvehiclesNoBorder;
    
    % Blocked vehicles (removing border effect)
    outputValues.NblockedNoBorder = nnz(BRid(simValues.IDvehicle(indexNoBorder),1)<0);
    outputValues.NblockedNoBorderTOT = outputValues.NblockedNoBorderTOT + outputValues.NblockedNoBorder;
    
    % Call function to create Awareness Matrix (with and removing border effect)
    % [#Correctly decoded beacons, #Errors, #Blocked neighbors, #Neighbors]
    awarenessMatrix = counter(simValues.IDvehicle,awarenessBRid,errorMatrix);
    awarenessMatrixNoBorder = awarenessMatrix(indexNoBorder,:);
    
    % Number of neighbors (removing border effect)
    outputValues.NneighboursNoBorder = sum(awarenessMatrixNoBorder(:,4));
    outputValues.NneighborsNoBorderTOT = outputValues.NneighborsNoBorderTOT + outputValues.NneighboursNoBorder;
    outputValues.StDevNeighboursNoBorder = std(awarenessMatrixNoBorder(:,4));
    outputValues.StDevNeighboursNoBorderTOT = outputValues.StDevNeighboursNoBorderTOT + outputValues.StDevNeighboursNoBorder;
    
    % Number of errors (removing border effect)
    outputValues.NerrorsNoBorder = length(errorMatrixNoBorder(:,1));
    outputValues.NerrorsNoBorderTOT = outputValues.NerrorsNoBorderTOT + outputValues.NerrorsNoBorder;
    
    % Number of received beacons (correct + errors) (removing border effect)
    outputValues.NreceivedBeaconsNoBorder = sum(awarenessMatrixNoBorder(:,1)) + sum(awarenessMatrixNoBorder(:,2));
    outputValues.NreceivedBeaconsNoBorderTOT = outputValues.NreceivedBeaconsNoBorderTOT + outputValues.NreceivedBeaconsNoBorder;
    
    % Number of correctly received beacons (removing border effect)
    outputValues.NcorrectlyReceivedBeaconsNoBorder = sum(awarenessMatrixNoBorder(:,1));
    outputValues.NcorrectlyReceivedBeaconsNoBorderTOT = outputValues.NcorrectlyReceivedBeaconsNoBorderTOT + outputValues.NcorrectlyReceivedBeaconsNoBorder;
    
    %     % Compute update delay (if enabled)
    %     if outParams.printUpdateDelay
    %         [simValues.updateTimeMatrix,outputValues.updateDelayCounter] = countUpdateDelay(simValues.IDvehicle,BRid,appParams.NbeaconsT,awarenessID,errorMatrix,elapsedTime,simValues.updateTimeMatrix,outputValues.updateDelayCounter,outParams.delayResolution);
    %     end
    if outParams.printUpdateDelay
        if elapsedTime >= 50.0
            [simValues.updateTimeMatrix,outputValues.updateDelayCounter, sumOfUpdateDelayForDistance, numOfUpdateForDistance, numOfTx, numOfRx, windowForUniqueVehicle,...
                RatenumOfTx, RatenumOfRx, PowernumOfTx, PowernumOfRx, RatesumOfUpdateDelayForDistance, RatenumOfUpdateForDistance, PowersumOfUpdateDelayForDistance, PowernumOfUpdateForDistance,...
                tmpRatenumOfTx, tmpRatenumOfRx, tmpPowernumOfTx, tmpPowernumOfRx, tmpRatesumOfUpdateDelayForDistance, tmpRatenumOfUpdateForDistance, tmpPowersumOfUpdateDelayForDistance, tmpPowernumOfUpdateForDistance]...
                = countUpdateDelay(simValues.IDvehicle,BRid,appParams.NbeaconsT,awarenessID,errorMatrix,elapsedTime,simValues.updateTimeMatrix,outputValues.updateDelayCounter,outParams.delayResolution, ...
                sumOfUpdateDelayForDistance, numOfUpdateForDistance, numOfTx, numOfRx, simValues.Xvehicle, simValues.Yvehicle, phyParams.Raw, phyParams.rangeForVehicleDensity, simParams.rho, phyParams.MCS, phyParams.smoothingFactorForITT, simParams.printLOG, simParams.ITTpercent, windowForUniqueVehicle, powerVehicle, ...
                RatenumOfTx, RatenumOfRx, PowernumOfTx, PowernumOfRx, RatesumOfUpdateDelayForDistance, RatenumOfUpdateForDistance, PowersumOfUpdateDelayForDistance, PowernumOfUpdateForDistance, ...
                tmpRatenumOfTx, tmpRatenumOfRx, tmpPowernumOfTx, tmpPowernumOfRx, tmpRatesumOfUpdateDelayForDistance, tmpRatenumOfUpdateForDistance, tmpPowersumOfUpdateDelayForDistance, tmpPowernumOfUpdateForDistance, appParams.ratio);
        end
    end
    
    % Compute packet delay (if enabled)
    if outParams.printPacketDelay
        outputValues.packetDelayCounter = countPacketDelay(simValues.IDvehicle,BRid,appParams.Tbeacon,appParams.NbeaconsT,elapsedTime,timeNextPacket,awarenessMatrix(:,4),errorMatrix(:,2),outputValues.packetDelayCounter,outParams.delayResolution);
    end
    
    % Compute power control allocation (if enabled)
    if outParams.printPowerControl
        % Convert linear PtxERP values to Ptx in dBm
        Ptx_dBm = 10*log10((phyParams.PtxERP_RB*appParams.RBsBeacon)/(2*phyParams.Gt))+30;
        outputValues.powerControlCounter = countPowerControl(BRid,Ptx_dBm,outputValues.powerControlCounter,outParams.powerResolution);
    end
    
    % Print number of neighbors per vehicle to file (if enabled)
    if outParams.printNeighbors
        printNeighborsToFile(awarenessMatrixNoBorder(:,4),outParams);
    end
    
    % Count details for distances up to the maximum awareness range (if enabled)
    if outParams.printDistanceDetails
        outputValues.distanceDetailsCounter = countDistanceDetails(neighborsDistance,neighborsBRid,errorMatrixRawMaxNoBorder,outputValues.distanceDetailsCounter,indexNoBorder);
    end
    
    % Update matrices needed for PRRmap creation in urban scenarios (if enabled)
    if outParams.printPRRmap
        simValues = counterMap(simValues,awarenessMatrix);
    end
    
    %% Update time next packet and rate control
    if simParams.rateControl
        [BRid, BRpassCounter, BRidAfterPass, BRidFake ,timeNextPacket, ITT] = computeBRforNext(simValues, phyParams, simValues.CBPMatrix(:, snap), snap, simParams.rho, phyParams.smoothingFactorForITT, simParams.ITTpercent, simParams.printLOG, slopeDefault, ITT, ITT_x, ITT_y, BRid, BRpassCounter, BRidAfterPass, timeNextPacket, elapsedTime, simParams, appParams, BRidFake, powerVehicle, rateVehicle, appParams.ratio);
    else
        % for only PowerControl - which is exactly moved 0.1
%         timeNextPacket = timeNextPacket + appParams.Tbeacon;
    end
    
    %% Radio Resources Reassignment
    
    if simParams.BRAlgorithm==2 || simParams.BRAlgorithm==6 || simParams.BRAlgorithm==7
        
        % Current scheduled reassign period
        reassignPeriod = mod(round(elapsedTime/appParams.Tbeacon)-1,NScheduledReassign)+1;
        
        % Find Index of vehicles which will perform sensing (BRAlgorithm=6)
        scheduledIndex = find(scheduledReassign(simValues.IDvehicle)==reassignPeriod);
        
        % Find IDs of vehicles whose resource will be reassigned
        scheduledID = simValues.IDvehicle(scheduledReassign(simValues.IDvehicle)==reassignPeriod);
        
    end
    
    if simParams.BRAlgorithm==1
        
        % BRs reassignment (CONTROLLED)
        % Call function for BRs reassignment
        % Returns updated BRid vector and number of successful reassignments
        alreadyReassigned = 0;
        [BRid,~,NreassignNoBorder,~,NunlockedNoBorder] = BRreassignmentControlled(simValues.IDvehicle,alreadyReassigned,errorMatrix,distance,BRid,appParams.Nbeacons,indexNoBorder,phyParams.Rreuse,simParams.randomOrder);
        
    elseif simParams.BRAlgorithm==2
        
        % BRs reassignment (CONTROLLED with SCHEDULED REASSIGNMENT)
        [BRid,~,NreassignNoBorder,~,NunlockedNoBorder] = BRreassignmentControlledScheduled(simValues.IDvehicle,BRid,scheduledID,distance,errorMatrix,appParams.Nbeacons,indexNoBorder,phyParams.Rreuse,simParams.randomOrder);
        
    elseif simParams.BRAlgorithm==3
        
        % BRs reassignment (AUTONOMOUS with SENSING RANGE)
        % Vehicles that cannot decode beacons advise transmitting vehicles
        % Vehicles search for an available BR in their sensing range (defined by design)
        % Returns updated BRid vector and number of successful reassignments
        [BRid,~,NreassignNoBorder,~,NunlockedNoBorder] = BRreassignmentAutoSense(simValues.IDvehicle,errorMatrix,distanceReal,BRid,appParams.Nbeacons,indexNoBorder,phyParams.Rsense);
        
    elseif simParams.BRAlgorithm==4
        
        % BRs reassignment (AUTONOMOUS with MAP-RP)
        % Vehicles read BR maps (2 bits per BR -> free, busy, error)
        % Vehicles search for an available BR collecting BR maps
        % Returns updated BRid vector and number of successful reassignments
        [BRid,~,NreassignNoBorder,~,NunlockedNoBorder] = BRreassignmentAutoMap(simValues.IDvehicle,errorMatrix,awarenessID,awarenessBRid,BRid,appParams.Nbeacons,indexNoBorder);
        
    elseif simParams.BRAlgorithm==5
        
        % BRs reassignment (AUTONOMOUS with SENSING ALGORITHM BY QUALCOMM INC.)
        % Vehicles consider resource reselection according to the design
        % parameters of the sensing algorithm (p,k,M)
        % Vehicles which perform sensing do not transmit for a beacon
        % period (BRid = -3)
        [BRid,oldBRid,~,NreassignNoBorder,~,NunlockedNoBorder] = BRreassignmentAutoQC(simValues.IDvehicle,BRid,oldBRid,simParams.pReselect,simParams.kBest,simParams.hysteresysM,RXpower,appParams.Nbeacons,indexNoBorder,phyParams.PnRB);
        
    elseif simParams.BRAlgorithm==6
        
        % BRs reassignment (AUTONOMOUS with SENSING ALGORITHM BY INTEL CORP.)
        
        % Sensing-based BRs reassignment
        [BRid,~,NreassignNoBorder,~,NunlockedNoBorder] = BRreassignmentAutoIntel(simValues.IDvehicle,BRid,scheduledIndex,simParams.MBest,RXpower,appParams.Nbeacons,indexNoBorder,phyParams.PnRB);
        
    elseif simParams.BRAlgorithm==7
        
        % BRs reassignment (CONTROLLED with MAXIMUM REUSE DISTANCE)
        [BRid,~,NreassignNoBorder,~,NunlockedNoBorder] = BRreassignmentControlledMaxReuse(simValues.IDvehicle,BRid,scheduledID,allNeighborsID,appParams.NbeaconsT,appParams.NbeaconsF,indexNoBorder);
        
    elseif simParams.BRAlgorithm==8
        
        % BRs reassignment (3GPP MODE 4)
        %         [BRid,~,NreassignNoBorder,~,NunlockedNoBorder,sensingMatrix,knownUsedMatrix,resReselectionCounter] = BRreassignment3GPPmode4(simValues.IDvehicle,BRid,sensingMatrix,knownUsedMatrix,resReselectionCounter,neighborsID,errorSCImatrix,simParams,timeNextPacket,RXpower,phyParams.IBEmatrix,appParams.Nbeacons,appParams.NbeaconsF,appParams.NbeaconsT,indexNoBorder,phyParams.Ksi,phyParams.PtxERP_RB,phyParams.PnRB,appParams);
%         [BRid, BRidAfterPass, ~,NreassignNoBorder,~,NunlockedNoBorder,sensingMatrix,knownUsedMatrix,resReselectionCounter, BRidBackup] = BRreassignment3GPPmode4(BRid, BRidAfterPass,simValues.IDvehicle,sensingMatrix,knownUsedMatrix,resReselectionCounter,neighborsID,errorSCImatrix,simParams,timeNextPacket,RXpower,phyParams.IBEmatrix,appParams.Nbeacons,appParams.NbeaconsF,appParams.NbeaconsT,indexNoBorder,phyParams.Ksi,phyParams.PtxERP_RB,phyParams.PnRB,appParams, BRidBackup);
           [BRid, BRidAfterPass, BRidFake,~,NreassignNoBorder,~,NunlockedNoBorder,sensingMatrix,knownUsedMatrix,resReselectionCounter] = BRreassignment3GPPmode4(BRid, BRidAfterPass,simValues.IDvehicle,BRidFake,sensingMatrix,knownUsedMatrix,resReselectionCounter,neighborsID,errorSCImatrix,simParams,timeNextPacket,RXpower,phyParams.IBEmatrix,appParams.Nbeacons,appParams.NbeaconsF,appParams.NbeaconsT,indexNoBorder,phyParams.Ksi,phyParams.PtxERP_RB,phyParams.PnRB,appParams, snap, phyParams, simValues.RXpowerSum, TransmitFlag, BRpassCounter, distanceReal, nonUsed);

    elseif simParams.BRAlgorithm==9
        
        if mod(elapsedTime-appParams.Tbeacon,simParams.Treassign)==0
            
            % BRs reassignment (CONTROLLED with POWER CONTROL)
            [BRid,phyParams.PtxERP_RB,lambda,~,NreassignNoBorder,~,NunlockedNoBorder] = BRreassignmentControlledPC(simValues.IDvehicle,BRid,phyParams.PtxERP_RB,CHgain,awarenessID,indexNoBorder,appParams.Nbeacons,lambda,phyParams.gammaMin,phyParams.PnRB,simParams.blockTarget,phyParams.maxPtxERP_RB);
            
        else
            NreassignNoBorder = 0;
            NunlockedNoBorder = 0;
        end
        
    elseif simParams.BRAlgorithm==101
        
        % Call Benchmark Algorithm 101 (RANDOM ALLOCATION)
        [BRid,~,NreassignNoBorder,~,NunlockedNoBorder] = BRreassignmentRandom(simValues.IDvehicle,BRid,appParams.Nbeacons,indexNoBorder);
        
    elseif simParams.BRAlgorithm==102
        
        % Call Benchmark Algorithm 102 (ORDERED ALLOCATION)
        [BRid,~,NreassignNoBorder,~,NunlockedNoBorder] = BRreassignmentOrdered(simValues.XvehicleReal,simValues.IDvehicle,BRid,appParams.Nbeacons,indexNoBorder);
        
    end
    
    % Incremental sum of successfully reassigned and unlocked vehicles
    outputValues.NreassignNoBorderTOT = outputValues.NreassignNoBorderTOT + NreassignNoBorder;
    outputValues.NunlockedNoBorderTOT = outputValues.NunlockedNoBorderTOT + NunlockedNoBorder;
    
    
    %%
    if ~simParams.printLOG
        %Information Age
        UpdateDelayForDistance = sumOfUpdateDelayForDistance ./ numOfUpdateForDistance;
        outFile = fopen(sprintf("./ITTpercent_%d/new_%d/IA_Raw%d_VDrange%d_rho%d_MCS%d_%f_ITTpercent_%d.data", simParams.ITTpercent, appParams.ratio, phyParams.Raw, phyParams.rangeForVehicleDensity, simParams.rho, phyParams.MCS, phyParams.smoothingFactorForITT, simParams.ITTpercent), 'a');
        for i = 1:length(UpdateDelayForDistance)
            fprintf(outFile, '%f\t',  UpdateDelayForDistance(i));
        end
        fprintf(outFile, '\n');
        fclose(outFile);
        
        %rate information age
        RateUpdateDelayForDistance = RatesumOfUpdateDelayForDistance ./ RatenumOfUpdateForDistance;
        outFile = fopen(sprintf("./ITTpercent_%d/new_%d/IA_Rate_Raw%d_VDrange%d_rho%d_MCS%d_%f_ITTpercent_%d.data", simParams.ITTpercent, appParams.ratio,phyParams.Raw, phyParams.rangeForVehicleDensity, simParams.rho, phyParams.MCS, phyParams.smoothingFactorForITT, simParams.ITTpercent), 'a');
        for i = 1:length(RateUpdateDelayForDistance)
            fprintf(outFile, '%f\t',  RateUpdateDelayForDistance(i));
        end
        fprintf(outFile, '\n');
        fclose(outFile);
        
        %tmp rate information age
        tmpRateUpdateDelayForDistance = tmpRatesumOfUpdateDelayForDistance ./ tmpRatenumOfUpdateForDistance;
        outFile = fopen(sprintf("./ITTpercent_%d/new_%d/IA_Rate_tmp_Raw%d_VDrange%d_rho%d_MCS%d_%f_ITTpercent_%d.data", simParams.ITTpercent, appParams.ratio, phyParams.Raw, phyParams.rangeForVehicleDensity, simParams.rho, phyParams.MCS, phyParams.smoothingFactorForITT, simParams.ITTpercent), 'a');
        for i = 1:length(tmpRateUpdateDelayForDistance)
            fprintf(outFile, '%f\t',  tmpRateUpdateDelayForDistance(i));
        end
        fprintf(outFile, '\n');
        fclose(outFile);
        
        %power information age
        PowerUpdateDelayForDistance = PowersumOfUpdateDelayForDistance ./ PowernumOfUpdateForDistance;
        outFile = fopen(sprintf("./ITTpercent_%d/new_%d/IA_Power_Raw%d_VDrange%d_rho%d_MCS%d_%f_ITTpercent_%d.data", simParams.ITTpercent, appParams.ratio, phyParams.Raw, phyParams.rangeForVehicleDensity, simParams.rho, phyParams.MCS, phyParams.smoothingFactorForITT, simParams.ITTpercent), 'a');
        for i = 1:length(PowerUpdateDelayForDistance)
            fprintf(outFile, '%f\t',  PowerUpdateDelayForDistance(i));
        end
        fprintf(outFile, '\n');
        fclose(outFile);      
        
        %tmp power information age
        tmpPowerUpdateDelayForDistance = tmpPowersumOfUpdateDelayForDistance ./ tmpPowernumOfUpdateForDistance;
        outFile = fopen(sprintf("./ITTpercent_%d/new_%d/IA_Power_tmp_Raw%d_VDrange%d_rho%d_MCS%d_%f_ITTpercent_%d.data", simParams.ITTpercent, appParams.ratio, phyParams.Raw, phyParams.rangeForVehicleDensity, simParams.rho, phyParams.MCS, phyParams.smoothingFactorForITT, simParams.ITTpercent), 'a');
        for i = 1:length(tmpPowerUpdateDelayForDistance)
            fprintf(outFile, '%f\t',  tmpPowerUpdateDelayForDistance(i));
        end
        fprintf(outFile, '\n');
        fclose(outFile); 

        
        % PRR
        PRRForDistance = numOfRx ./ numOfTx;
        outFile = fopen(sprintf("./ITTpercent_%d/new_%d/PRR_Raw%d_VDrange%d_rho%d_MCS%d_%f_ITTpercent_%d.data", simParams.ITTpercent, appParams.ratio, phyParams.Raw, phyParams.rangeForVehicleDensity, simParams.rho, phyParams.MCS, phyParams.smoothingFactorForITT, simParams.ITTpercent), 'a');
        for i = 1:length(PRRForDistance)
            fprintf(outFile, '%f\t',  PRRForDistance(i));
        end
        fprintf(outFile, '\n');
        fclose(outFile);
        
        % rate PRR
        RatePRRForDistance = RatenumOfRx ./ RatenumOfTx;
        outFile = fopen(sprintf("./ITTpercent_%d/new_%d/PRR_Rate_Raw%d_VDrange%d_rho%d_MCS%d_%f_ITTpercent_%d.data", simParams.ITTpercent, appParams.ratio, phyParams.Raw, phyParams.rangeForVehicleDensity, simParams.rho, phyParams.MCS, phyParams.smoothingFactorForITT, simParams.ITTpercent), 'a');
        for i = 1:length(RatePRRForDistance)
            fprintf(outFile, '%f\t',  RatePRRForDistance(i));
        end
        fprintf(outFile, '\n');
        fclose(outFile);        
        
        % tmp rate PRR
        tmpRatePRRForDistance = tmpRatenumOfRx ./ tmpRatenumOfTx;
        outFile = fopen(sprintf("./ITTpercent_%d/new_%d/PRR_Rate_tmp_Raw%d_VDrange%d_rho%d_MCS%d_%f_ITTpercent_%d.data", simParams.ITTpercent, appParams.ratio, phyParams.Raw, phyParams.rangeForVehicleDensity, simParams.rho, phyParams.MCS, phyParams.smoothingFactorForITT, simParams.ITTpercent), 'a');
        for i = 1:length(tmpRatePRRForDistance)
            fprintf(outFile, '%f\t',  tmpRatePRRForDistance(i));
        end
        fprintf(outFile, '\n');
        fclose(outFile);
        
        % power PRR
        PowerPRRForDistance = PowernumOfRx ./ PowernumOfTx;
        outFile = fopen(sprintf("./ITTpercent_%d/new_%d/PRR_Power_Raw%d_VDrange%d_rho%d_MCS%d_%f_ITTpercent_%d.data", simParams.ITTpercent, appParams.ratio, phyParams.Raw, phyParams.rangeForVehicleDensity, simParams.rho, phyParams.MCS, phyParams.smoothingFactorForITT, simParams.ITTpercent), 'a');
        for i = 1:length(PowerPRRForDistance)
            fprintf(outFile, '%f\t',  PowerPRRForDistance(i));
        end
        fprintf(outFile, '\n');
        fclose(outFile);
        
        % tmp power PRR
        tmpPowerPRRForDistance = tmpPowernumOfRx ./ tmpPowernumOfTx;
        outFile = fopen(sprintf("./ITTpercent_%d/new_%d/PRR_Power_tmp_Raw%d_VDrange%d_rho%d_MCS%d_%f_ITTpercent_%d.data", simParams.ITTpercent, appParams.ratio, phyParams.Raw, phyParams.rangeForVehicleDensity, simParams.rho, phyParams.MCS, phyParams.smoothingFactorForITT, simParams.ITTpercent), 'a');
        for i = 1:length(tmpPowerPRRForDistance)
            fprintf(outFile, '%f\t',  tmpPowerPRRForDistance(i));
        end
        fprintf(outFile, '\n');
        fclose(outFile);
        
        %Xvehicle - location of each vehicle
        outFile = fopen(sprintf("./ITTpercent_%d/new_%d/XvehicleReal_Raw%d_VDrange%d_rho%d_MCS%d_%f_ITTpercent_%d.data", simParams.ITTpercent, appParams.ratio, phyParams.Raw, phyParams.rangeForVehicleDensity, simParams.rho, phyParams.MCS, phyParams.smoothingFactorForITT, simParams.ITTpercent), 'a');
        for i = 1:length(XvehicleRealOld)
            fprintf(outFile, '%f\t',  simValues.XvehicleReal(i));
        end
        fprintf(outFile, '\n');
        fclose(outFile);
                
        outFile = fopen(sprintf("./ITTpercent_%d/new_%d/resReselectionCounter_Raw%d_VDrange%d_rho%d_MCS%d_%f_ITTpercent_%d.data", simParams.ITTpercent, appParams.ratio, phyParams.Raw, phyParams.rangeForVehicleDensity, simParams.rho, phyParams.MCS, phyParams.smoothingFactorForITT, simParams.ITTpercent), 'a');
        for i = 1:length(XvehicleRealOld)
            fprintf(outFile, '%d\t',  resReselectionCounter(i));
        end
        fprintf(outFile, '\n');
        fclose(outFile);
        
        outFile = fopen(sprintf("./ITTpercent_%d/new_%d/BRidFake_Raw%d_VDrange%d_rho%d_MCS%d_%f_ITTpercent_%d.data", simParams.ITTpercent, appParams.ratio, phyParams.Raw, phyParams.rangeForVehicleDensity, simParams.rho, phyParams.MCS, phyParams.smoothingFactorForITT, simParams.ITTpercent), 'a');
        for i = 1:length(XvehicleRealOld)
            fprintf(outFile, '%d\t',  BRidFake(i));
        end
        fprintf(outFile, '\n');
        fclose(outFile);
        
        
        
        % SLT - 일단 그냥 PDR * ITT로 가쟈 - Tx와 Rx의 수 자체가 달라지는거지
        % 아 조금만 더 고민해보쟈.. 일단 Tx,Rx 수랑 ITT랑 거리 있으면 그래프는 따로 계산해서 그리면 됨
        % 고민하기 전에 돌렸어도 되는 거였네...
        
        % SLTForDistance = (0.3 / 40) * numOfRx;     %Kbps
        %
        % outFile = fopen(sprintf("SLT_Raw%d_VDrange%d_rho%d_MCS%d_%f.data", phyParams.Raw, phyParams.rangeForVehicleDensity, simParams.rho, phyParams.MCS, phyParams.smoothingFactorForITT), 'a');
        % for i = 1:length(SLTForDistance)
        %     fprintf(outFile, '%f\t',  SLTForDistance(i));
        % end
        % fprintf(outFile, '\n');
        % fclose(outFile);
    end
    
end

% Stop stopwatch
outputValues.computationTime = toc;

end