function phyParams = computeTransmissionPowerforNext(simValues, phyParams, currentCBP, snap, rho, smoothingFactorForITT, ITTpercent, printLOG)

% yeomyung
% function for Power Control
% depending on the current CBP

gCBP = ones(length(currentCBP),1);

%%  gCBP
%calculate gCBP
for  i = 1:length(currentCBP)
    if currentCBP(i) <= phyParams.minChanUtil
        gCBP(i) = phyParams.maxPtx_dBm;
    elseif currentCBP(i) >= phyParams.maxChanUtil
        gCBP(i) = phyParams.minPtx_dBm;
    else
        gCBP(i) = phyParams.minPtx_dBm + ((phyParams.maxPtx_dBm - phyParams.minPtx_dBm) * (phyParams.maxChanUtil - currentCBP(i)) / (phyParams.maxChanUtil - phyParams.minChanUtil));
    end
end

%% radiation power for next
% smoothingFactor==0.5
phyParams.Ptx_dBm_RB = phyParams.Ptx_dBm_RB + phyParams.smoothingFactor * (gCBP - phyParams.Ptx_dBm_RB);

for  i = 1:length(currentCBP)
    if phyParams.Ptx_dBm_RB(i) <= 10.5
        phyParams.Ptx_dBm_RB(i) = phyParams.minPtx_dBm;
    else
        phyParams.Ptx_dBm_RB(i) = round(phyParams.Ptx_dBm_RB(i)/1);
    end
end

%% print

if ~printLOG
    
    if mod(snap,10) == 0
        [~, sortedXVehiclesIndex] = sort(simValues.XvehicleReal);
        outFile2 = fopen(sprintf("./ITTpercent_%d/TxPowerHistory_Sort_Raw%d_VDrange%d_rho%d_MCS%d_%d.data", ITTpercent, phyParams.Raw, phyParams.rangeForVehicleDensity, rho, phyParams.MCS, smoothingFactorForITT),'a');
        for i = 1:length(phyParams.Ptx_dBm_RB)
            fprintf(outFile2, '%d\t', phyParams.Ptx_dBm_RB(sortedXVehiclesIndex(i)));
        end
        fprintf(outFile2, '\n');
        fclose(outFile2);
        
        outFile2 = fopen(sprintf("./ITTpercent_%d/TxPowerHistory_Raw%d_VDrange%d_rho%d_MCS%d_%d.data", ITTpercent, phyParams.Raw, phyParams.rangeForVehicleDensity, rho, phyParams.MCS, smoothingFactorForITT),'a');
        for i = 1:length(phyParams.Ptx_dBm_RB)
            fprintf(outFile2, '%d\t', phyParams.Ptx_dBm_RB(i));
        end
        fprintf(outFile2, '\n');
        fclose(outFile2);
    end
end
end

