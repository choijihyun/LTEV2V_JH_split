function [BRid, BRpassCounter, BRidAfterPass, BRidFake, timeNextPacket, ITT] = computeBRforNext(simValues, phyParams, currentCBP, snap, rho, smoothingFactorForITT, ITTpercent, printLOG, slopeDefault, ITT, ITT_x, ITT_y,BRid, BRpassCounter, BRidAfterPass, timeNextPacket, elapsedTime, simParams, appParams, BRidFake)
% jihyun
% calculate ITT by CBP
% and use BRafterPass and BRpassCounter

slope = slopeDefault;
yIntercept = ITT_x - slope * 0.5;

for  i = 1 :length(timeNextPacket)
    eachCBP = currentCBP(i);
    
    if BRpassCounter(i) == 0 && BRid(i) > 0     
        % 이번 snap에 쏜 차량들
        if eachCBP <= phyParams.minChanUtil
            ITT(i) = ITT_x;
            minPass = round(ITT_x*10);
            BRpassCounter(i) = minPass;
            BRidAfterPass(i) = BRid(i);
            BRidFake(i) = BRid(i);
            BRid(i) = -2;
            
        elseif eachCBP >= phyParams.maxChanUtil
            ITT(i) = ITT_y;
            maxPass = round(ITT_y*10);
            BRpassCounter(i) = maxPass;
            BRidAfterPass(i) = BRid(i);
            BRidFake(i) = BRid(i);
            
            BRid(i) = -2;
            
        else 
            % 0.1단위로 전송할 수 있도록(안그러면 자원 위치가 바ㄱ뀌는데 그러면 당연히 충돌나겠지)
%             ITT(i) = round((slope * eachCBP + yIntercept), 1); 
%             BRpassCounter(i) = round(ITT(i)*10);
%             BRidAfterPass(i) = BRid(i);
%             BRidFake(i) = BRid(i);
%             BRid(i) = -2;
            
            ITT(i) = round((slope * eachCBP + yIntercept), 3);
            BRpassCounter(i) = floor(ITT(i)/appParams.Tbeacon);  % after the value of BRpassCounter snap is pass
            
            % 理쒕? 99移? 諛뽰뿉 紐삳뒛?뼱?굹!
            BRidAfterPass(i) = BRid(i) + round(mod(ITT(i),appParams.Tbeacon)*appParams.Nbeacons/2*10);   % using this BR after BRpassCounter
            
            if (BRid(i)<=appParams.Nbeacons/2 && BRidAfterPass(i) > appParams.Nbeacons/2)...
                    || (BRid(i)<=appParams.Nbeacons && BRidAfterPass(i) > appParams.Nbeacons)     % 원래 오른쪽 끝에 있어서 한 턴 더 넘어가게 되는 경우
                BRpassCounter(i) = BRpassCounter(i) + 1;
                BRidAfterPass(i) = BRidAfterPass(i) - appParams.Nbeacons/2;   
            end
            % jihyun - for maintain BRid in this snap
            BRidFake(i) = BRid(i);
            BRid(i) = -2;
            
        end 
        
        if timeNextPacket(i) < elapsedTime
            timeNextPacket(i) = timeNextPacket(i) + ITT(i); % using number of BR moved
        end
    end
    

    
    if BRpassCounter(i) ~= 0
        %jihyun
        
        if BRpassCounter(i) == 1 && elapsedTime > 0.1
            % 다음 snap에 쏘는 차량들
            BRid(i) = BRidAfterPass(i);
            BRidAfterPass(i) = -2;
            BRidFake(i) = BRid(i);
        end
        BRpassCounter(i) = BRpassCounter(i) - 1;
        
    end
end

% print
if ~simParams.printLOG
    [~, sortedXVehiclesIndex] = sort(simValues.XvehicleReal);
    outFile3 = fopen(sprintf("./ITTpercent_%d/ITTHistory_Sort_Raw%d_VDrange%d_rho%d_MCS%d_%d.data", simParams.ITTpercent, phyParams.Raw, phyParams.rangeForVehicleDensity, simParams.rho, phyParams.MCS, phyParams.smoothingFactorForITT),'a');
    outFile4 = fopen(sprintf("./ITTpercent_%d/BRpassCounter_Sort_Raw%d_VDrange%d_rho%d_MCS%d_%d.data", simParams.ITTpercent, phyParams.Raw, phyParams.rangeForVehicleDensity, simParams.rho, phyParams.MCS, phyParams.smoothingFactorForITT),'a');
    for  i = 1 :length(timeNextPacket)
        fprintf(outFile3, '%f\t',  ITT(sortedXVehiclesIndex(i)));
        fprintf(outFile4, '%d\t',  BRpassCounter(sortedXVehiclesIndex(i)));
    end
    fprintf(outFile3, '\n');
    fprintf(outFile4, '\n');
    fclose(outFile3);
    fclose(outFile4);
    
    outFile3 = fopen(sprintf("./ITTpercent_%d/ITTHistory_Raw%d_VDrange%d_rho%d_MCS%d_%d.data", simParams.ITTpercent, phyParams.Raw, phyParams.rangeForVehicleDensity, simParams.rho, phyParams.MCS, phyParams.smoothingFactorForITT),'a');
    outFile4 = fopen(sprintf("./ITTpercent_%d/BRpassCounter_Raw%d_VDrange%d_rho%d_MCS%d_%d.data", simParams.ITTpercent, phyParams.Raw, phyParams.rangeForVehicleDensity, simParams.rho, phyParams.MCS, phyParams.smoothingFactorForITT),'a');
    for  i = 1 :length(timeNextPacket)
        fprintf(outFile3, '%f\t',  ITT(i));
        fprintf(outFile4, '%d\t',  BRpassCounter(i));
    end
    fprintf(outFile3, '\n');
    fprintf(outFile4, '\n');
    
end

end

