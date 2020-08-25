function [RXpowerSum, CBPMatrix, RXpowerHist, CBPMatrixHist, TransmitFlag, nonUsed, totalUse] = calculateCBP(CBPMatrixHist, simValues, CBPMatrix, IDvehicle, NbeaconsT, ...
    RXpower, distance, neighborsID, allNeighborsID, BRid, snap, RXpowerSum,...
    CBPThresh, appParams, RXpowerHist, Nbeacons,...
    NbeaconsF, CBRRange, timeNextPacket, elapsedTime, Raw, rangeForVehicleDensity, rho, MCS, smoothingFactorForITT, ITTpercent, printLOG, BRidFake, IBEmatrix,PnRB, TransmitFlag, distanceReal, totalUse, powerVehicle, rateVehicle, ratio)


RXpowerSum = 0.0 .* RXpowerSum;     % initiate

RXpowerHist(:, 1:Nbeacons*10) = RXpowerHist(:,Nbeacons+1:Nbeacons*11);   % circshift 0.1s 1~2000 <= 201~2200

% find Tx vehicle to calculate RxPower & not used resource
arr = zeros(200,1);
for i=1:length(BRid)
    if BRid(i)>0
        arr(BRid(i)) = arr(BRid(i))+1;
        totalUse(BRid(i)) = totalUse(BRid(i)) + 1;
    end
end
nonUsed = find(arr==0);


%jihyun- flag for transmit vehicles-> this process is required because BRid is change in rate control
% this variable will be used in reassignment
% 0: BRid==-2, nonUsed
% 1: BRid>0, which means transmit in this snap
% 2: BRid==-1
TransmitFlag = ones(simValues.maxID,1);
TransmitFlag(BRid==-2)=0;
TransmitFlag(BRid==-1)=2;

% find nodes that transmit in this snap
[reservedNodes] = find(BRid>0);
for i=1:length(reservedNodes)
    RXpowerSum(:,BRid(reservedNodes(i))) = RXpowerSum(:,BRid(reservedNodes(i))) + RXpower(reservedNodes(i),:)';
    %jihyun - in same subframe, two subchannel affect each other with 0.0035
    % Find total received power using IBE
    IBE = IBEmatrix(1,2);
    if BRid(reservedNodes(i)) > 100
        %         RXpowerSum(reservedNodes(i),BRid(reservedNodes(i)) - 100) =  RXpower(reservedNodes(i),reservedNodes(i));
        RXpowerSum(:,BRid(reservedNodes(i))-100) = RXpowerSum(:,BRid(reservedNodes(i))-100) + RXpower(reservedNodes(i),:)'*IBE;
    else
        %         RXpowerSum(reservedNodes(i),BRid(reservedNodes(i)) + 100) =  RXpower(reservedNodes(i),reservedNodes(i));
        RXpowerSum(:,BRid(reservedNodes(i)) + 100) =  RXpowerSum(:,BRid(reservedNodes(i)) + 100)+ RXpower(reservedNodes(i),:)'*IBE;
    end
end


% RXpowerHist 끝에 추가 - 2001부터 2200에 RXpowerSum을 대입
RXpowerHist(:, Nbeacons*10+1:Nbeacons*11) = RXpowerSum;

%%
%100ms
% CBPMatrix(:,snap) = sum(RXpowerSum > CBPThresh,2) / (NbeaconsT*NbeaconsF);

% Calculate the beacon resources used in the time and frequency domains
% Find not assigned BRid(BRidFake maintain value of BRid when passcounter~=0(BRid==-2))
NOT = BRidFake<0;
% Calculate BRidT = BRid
BRidTT = BRidFake;
BRidTT(NOT) = -1;

IDlength = length(IDvehicle);
if snap < 3
    % rawCBP
    CBPMatrix(:,snap) = sum(RXpowerSum > CBPThresh,2) / (NbeaconsT*NbeaconsF);
else
    for i=1:IDlength
        if BRidTT(i) == -1
            %             for not assigned BRid
            CBPMatrix(i,snap) = sum(RXpowerHist( i , Nbeacons* 10+1 : (Nbeacons*11)) > CBPThresh,2) / (NbeaconsT*NbeaconsF);
        else
            %             Raw CBP calculate
            CBPMatrix(i,snap) = sum(RXpowerHist( i ,( Nbeacons*10+BRidTT(i) - (Nbeacons) +1) : ( Nbeacons*10+BRidTT(i) )) > CBPThresh,2) / (NbeaconsT*NbeaconsF);
        end
    end
end

% update CBPMatrixHistory for next snap calculate
% CBPMatrix: Calculated, CBPMatrixHist: RawCBP
CBPMatrixHist(:, snap) = CBPMatrix(:, snap);

if snap>30
    CBPMatrix;
end


if snap>1
    CBPMatrix( : , snap) = (CBPMatrix( : , snap-1) + CBPMatrixHist(:, snap))/2;
end



% this is for 200ms calculate which is not matching BRidT
% CBPMatrix(:,snap) = sum(RXpowerHist(:,Nbeacons * (max(11-snap, 9)) + 1 : (Nbeacons*11)) > CBPThresh,2) / (NbeaconsT*NbeaconsF*min(snap,2));




%% print
if ~printLOG
    %total
    [~, sortedXVehiclesIndex] = sort(simValues.XvehicleReal);
    outFile = fopen(sprintf("./ITTpercent_%d/new_%d/CBPHistory_Sort_Raw%d_VDrange%d_rho%d_MCS%d_%d.data", ITTpercent, ratio, Raw, rangeForVehicleDensity, rho, MCS, smoothingFactorForITT),'a');
    for i = 1:length(CBPMatrix(:,snap))
        fprintf(outFile, '%f\t',  CBPMatrix(sortedXVehiclesIndex(i),snap));
    end
    fprintf(outFile, '\n');
    fclose(outFile);
    % total
    
    %log for smoothing cbp
    outFile = fopen(sprintf("./ITTpercent_%d/new_%d/CBPHistory_Raw%d_VDrange%d_rho%d_MCS%d_%d.data", ITTpercent, ratio, Raw, rangeForVehicleDensity, rho, MCS, smoothingFactorForITT),'a');
    for i = 1:length(CBPMatrix(:,snap))
        fprintf(outFile, '%f\t',  CBPMatrix(i,snap));
    end
    fprintf(outFile, '\n');
    fclose(outFile);
    
    %log for raw cbp
    outFile = fopen(sprintf("./ITTpercent_%d/new_%d/CBPMatrixHistory_Raw%d_VDrange%d_rho%d_MCS%d_%d.data", ITTpercent, ratio, Raw, rangeForVehicleDensity, rho, MCS, smoothingFactorForITT),'a');
    for i = 1:length(CBPMatrixHist(:,snap))
        fprintf(outFile, '%f\t',  CBPMatrixHist(i,snap));
    end
    fprintf(outFile, '\n');
    fclose(outFile);
    
    outFile = fopen(sprintf("./ITTpercent_%d/new_%d/CBPMatrixHistory_Sort_Raw%d_VDrange%d_rho%d_MCS%d_%d.data", ITTpercent, ratio, Raw, rangeForVehicleDensity, rho, MCS, smoothingFactorForITT),'a');
    for i = 1:length(CBPMatrixHist(:,snap))
        fprintf(outFile, '%f\t',  CBPMatrixHist(sortedXVehiclesIndex(i),snap));
    end
    fprintf(outFile, '\n');
    fclose(outFile);
    
    %% for rate and power vehicle's cbp
    %log for smoothing cbp
    %power control vehicle cbp
    outFile = fopen(sprintf("./ITTpercent_%d/new_%d/CBPHistory_Raw%d_VDrange%d_rho%d_MCS%d_%d_power.data", ITTpercent, ratio, Raw, rangeForVehicleDensity, rho, MCS, smoothingFactorForITT),'a');
    for i = 1:powerVehicle
        fprintf(outFile, '%f\t',  CBPMatrix(i,snap));
    end
    fprintf(outFile, '\n');
    fclose(outFile);
    
    index = find(simValues.IDvehicle<=powerVehicle);
    [~, sortedXVehiclesIndex2] = sort(simValues.XvehicleReal(index));
    outFile = fopen(sprintf("./ITTpercent_%d/new_%d/CBPHistory_Sort_Raw%d_VDrange%d_rho%d_MCS%d_%d_power.data", ITTpercent, ratio, Raw, rangeForVehicleDensity, rho, MCS, smoothingFactorForITT),'a');
    for i = 1:powerVehicle
        fprintf(outFile, '%f\t',  CBPMatrix(index(sortedXVehiclesIndex2(i)),snap));
    end
    fprintf(outFile, '\n');
    fclose(outFile);
    
    %rate control vehicle
    outFile = fopen(sprintf("./ITTpercent_%d/new_%d/CBPHistory_Raw%d_VDrange%d_rho%d_MCS%d_%d_rate.data", ITTpercent, ratio, Raw, rangeForVehicleDensity, rho, MCS, smoothingFactorForITT),'a');
    for i = powerVehicle+1:length(CBPMatrix(:,snap))
        fprintf(outFile, '%f\t',  CBPMatrix(i,snap));
    end
    fprintf(outFile, '\n');
    fclose(outFile);
    
    index2 = find(simValues.IDvehicle>powerVehicle);
    [~, sortedXVehiclesIndex3] = sort(simValues.XvehicleReal(index2));
    outFile = fopen(sprintf("./ITTpercent_%d/new_%d/CBPHistory_Sort_Raw%d_VDrange%d_rho%d_MCS%d_%d_rate.data", ITTpercent, ratio, Raw, rangeForVehicleDensity, rho, MCS, smoothingFactorForITT),'a');
    for i = 1:rateVehicle
        fprintf(outFile, '%f\t',  CBPMatrix(index2(sortedXVehiclesIndex3(i)),snap));
    end
    fprintf(outFile, '\n');
    fclose(outFile);
    
    
    %% for detail logging such as BRid status
    outFile = fopen(sprintf("./ITTpercent_%d/new_%d/TrackBRid_Raw%d_VDrange%d_rho%d_MCS%d_%d.data", ITTpercent, ratio, Raw, rangeForVehicleDensity, rho, MCS, smoothingFactorForITT),'a');
    for i = 1:length(BRid)
        fprintf(outFile, '%d\t',  BRid(i));
    end
    fprintf(outFile, '\n');
    fclose(outFile);
    
    outFile = fopen(sprintf("./ITTpercent_%d/new_%d/TrackBRidFake_Raw%d_VDrange%d_rho%d_MCS%d_%d.data", ITTpercent, ratio, Raw, rangeForVehicleDensity, rho, MCS, smoothingFactorForITT),'a');
    for i = 1:length(BRidFake)
        fprintf(outFile, '%d\t',  BRidFake(i));
    end
    fprintf(outFile, '\n');
    fclose(outFile);
    
    outFile = fopen(sprintf("./ITTpercent_%d/new_%d/BRid_Raw%d_VDrange%d_rho%d_MCS%d_%d.data", ITTpercent, ratio, Raw, rangeForVehicleDensity, rho, MCS, smoothingFactorForITT),'a');
    for i = 1:length(arr)
        fprintf(outFile, '%d\t',  arr(i));
    end
    fprintf(outFile, '\n');
    fclose(outFile);
    
    outFile = fopen(sprintf("./ITTpercent_%d/new_%d/nonUsed_Raw%d_VDrange%d_rho%d_MCS%d_%d.data", ITTpercent, ratio, Raw, rangeForVehicleDensity, rho, MCS, smoothingFactorForITT),'a');
    for i = 1:length(nonUsed)
        fprintf(outFile, '%d\t',  nonUsed(i));
    end
    fprintf(outFile, '\n');
    fclose(outFile);
end
end