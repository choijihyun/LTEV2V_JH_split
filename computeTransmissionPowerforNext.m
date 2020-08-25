function phyParams = computeTransmissionPowerforNext(simValues, phyParams, currentCBP, snap, rho, smoothingFactorForITT, ITTpercent, printLOG, powerVehicle, rateVehicle, ratio)

% yeomyung
% function for Power Control
% depending on the current CBP

gCBP = ones(powerVehicle,1);

%%  gCBP
% %calculate gCBP
% for  i = 1:powerVehicle
%     if currentCBP(i) <= phyParams.minChanUtil
%         gCBP(i) = phyParams.maxPtx_dBm;
%     elseif currentCBP(i) >= phyParams.maxChanUtil
%         gCBP(i) = phyParams.minPtx_dBm;
%     else
%         gCBP(i) = phyParams.minPtx_dBm + ((phyParams.maxPtx_dBm - phyParams.minPtx_dBm) * (phyParams.maxChanUtil - currentCBP(i)) / (phyParams.maxChanUtil - phyParams.minChanUtil));
%     end
% end

%교수님께서 새롭게 주신 식
% power(cbp)=20/(1+exp(0.4*(cbp-65))) dBm
for  i = 1:powerVehicle
    gCBP(i) = 20/(1+exp(0.4*((currentCBP(i)-0.65)*100)));
end

%% radiation power for next
% smoothingFactor==0.5
%     phyParams.Ptx_dBm_RB = phyParams.Ptx_dBm_RB + phyParams.smoothingFactor * (gCBP - phyParams.Ptx_dBm_RB);
for i=1:powerVehicle
    phyParams.Ptx_dBm_RB(i) = phyParams.Ptx_dBm_RB(i) + phyParams.smoothingFactor * (gCBP(i) - phyParams.Ptx_dBm_RB(i));
end

% for  i = 1:powerVehicle
%     if phyParams.Ptx_dBm_RB(i) <= 10.5
%         phyParams.Ptx_dBm_RB(i) = phyParams.minPtx_dBm;
%     else
%         phyParams.Ptx_dBm_RB(i) = round(phyParams.Ptx_dBm_RB(i)/1);
%     end
% end

for  i = 1:powerVehicle
    phyParams.Ptx_dBm_RB(i) = round(phyParams.Ptx_dBm_RB(i)/1);
end

%% print

if ~printLOG
    %sort power control TX
    index = find(simValues.IDvehicle<=powerVehicle);
    [~, sortedXVehiclesIndex] = sort(simValues.XvehicleReal(index));
    outFile2 = fopen(sprintf("./ITTpercent_%d/new_%d/TxPowerHistory_Sort_Raw%d_VDrange%d_rho%d_MCS%d_%d_power.data", ITTpercent, ratio, phyParams.Raw, phyParams.rangeForVehicleDensity, rho, phyParams.MCS, smoothingFactorForITT),'a');
    for i = 1:powerVehicle
        fprintf(outFile2, '%d\t', phyParams.Ptx_dBm_RB(index(sortedXVehiclesIndex(i))));
    end
    fprintf(outFile2, '\n');
    fclose(outFile2);
    
    %raw
    %         [~, sortedXVehiclesIndex] = sort(simValues.XvehicleReal);
    outFile2 = fopen(sprintf("./ITTpercent_%d/new_%d/TxPowerHistory_Raw%d_VDrange%d_rho%d_MCS%d_%d_power.data", ITTpercent, ratio, phyParams.Raw, phyParams.rangeForVehicleDensity, rho, phyParams.MCS, smoothingFactorForITT),'a');
    for i = 1:powerVehicle
        fprintf(outFile2, '%d\t', phyParams.Ptx_dBm_RB(i));
    end
    fprintf(outFile2, '\n');
    fclose(outFile2);
    
    %rate Tx
    %         [~, sortedXVehiclesIndex] = sort(simValues.XvehicleReal);\
    index2 = find(simValues.IDvehicle>powerVehicle);
    [~, sortedXVehiclesIndex2] = sort(simValues.XvehicleReal(index2));
    outFile2 = fopen(sprintf("./ITTpercent_%d/new_%d/TxPowerHistory_Sort_Raw%d_VDrange%d_rho%d_MCS%d_%d_rate.data", ITTpercent, ratio, phyParams.Raw, phyParams.rangeForVehicleDensity, rho, phyParams.MCS, smoothingFactorForITT),'a');
    for i = 1:rateVehicle
        fprintf(outFile2, '%d\t', phyParams.Ptx_dBm_RB(index2(sortedXVehiclesIndex2(i))));
    end
    fprintf(outFile2, '\n');
    fclose(outFile2);
    
    %raw
    outFile2 = fopen(sprintf("./ITTpercent_%d/new_%d/TxPowerHistory_Raw%d_VDrange%d_rho%d_MCS%d_%d_rate.data", ITTpercent, ratio, phyParams.Raw, phyParams.rangeForVehicleDensity, rho, phyParams.MCS, smoothingFactorForITT),'a');
    for i = powerVehicle+1:length(phyParams.Ptx_dBm_RB)
        fprintf(outFile2, '%d\t', phyParams.Ptx_dBm_RB(i));
    end
    fprintf(outFile2, '\n');
    fclose(outFile2);
    
    [~, sortedXVehiclesIndex3] = sort(simValues.XvehicleReal);
    outFile2 = fopen(sprintf("./ITTpercent_%d/new_%d/TxPowerHistory_Sort_Raw%d_VDrange%d_rho%d_MCS%d_%d.data", ITTpercent, ratio, phyParams.Raw, phyParams.rangeForVehicleDensity, rho, phyParams.MCS, smoothingFactorForITT),'a');
    for i = 1:length(phyParams.Ptx_dBm_RB)
        fprintf(outFile2, '%d\t', phyParams.Ptx_dBm_RB(sortedXVehiclesIndex3(i)));
    end
    fprintf(outFile2, '\n');
    fclose(outFile2);
    
    outFile2 = fopen(sprintf("./ITTpercent_%d/new_%d/TxPowerHistory_Raw%d_VDrange%d_rho%d_MCS%d_%d.data", ITTpercent, ratio, phyParams.Raw, phyParams.rangeForVehicleDensity, rho, phyParams.MCS, smoothingFactorForITT),'a');
    for i = 1:length(phyParams.Ptx_dBm_RB)
        fprintf(outFile2, '%d\t', phyParams.Ptx_dBm_RB(i));
    end
    fprintf(outFile2, '\n');
    fclose(outFile2);
end
end

