function [updateTimeMatrix,updateDelayCounter, sumOfUpdateDelayForDistance, numOfUpdateForDistance, numOfTx, numOfRx, windowForUniqueVehicle,...
    RatenumOfTx, RatenumOfRx, PowernumOfTx, PowernumOfRx, RatesumOfUpdateDelayForDistance, RatenumOfUpdateForDistance, PowersumOfUpdateDelayForDistance, PowernumOfUpdateForDistance,...
    tmpRatenumOfTx, tmpRatenumOfRx, tmpPowernumOfTx, tmpPowernumOfRx, tmpRatesumOfUpdateDelayForDistance, tmpRatenumOfUpdateForDistance, tmpPowersumOfUpdateDelayForDistance, tmpPowernumOfUpdateForDistance]...
    = countUpdateDelay(IDvehicle,BRid,NbeaconsT,awarenessID,errorMatrix,elapsedTime,updateTimeMatrix,updateDelayCounter,delayResolution, ...
    sumOfUpdateDelayForDistance, numOfUpdateForDistance, numOfTx, numOfRx, Xvehicle, Yvehicle, Raw, rangeForVehicleDensity, rho, MCS, smoothingFactorForITT, printLOG, ITTpercent, windowForUniqueVehicle, powerVehicle,...
    RatenumOfTx, RatenumOfRx, PowernumOfTx, PowernumOfRx, RatesumOfUpdateDelayForDistance, RatenumOfUpdateForDistance, PowersumOfUpdateDelayForDistance, PowernumOfUpdateForDistance, ...
    tmpRatenumOfTx, tmpRatenumOfRx, tmpPowernumOfTx, tmpPowernumOfRx, tmpRatesumOfUpdateDelayForDistance, tmpRatenumOfUpdateForDistance, tmpPowersumOfUpdateDelayForDistance, tmpPowernumOfUpdateForDistance)
% Function to compute the update delay between received beacons
% Returns the updated updateTimeMatrix and updateDelayCounter

% Update updateTimeMatrix -> matrix containing the timestamp of the last received beacon
% Row index -> receiving vehicle's ID
% Column index -> transmitting vehicle's ID
Nv = length(IDvehicle);
all = 1:length(BRid);
delayMax = length(updateDelayCounter)*delayResolution;

% Calculate BRidT = vector of BRid in the time domain
BRidT = mod(BRid-1,NbeaconsT)+1;

% Reset timestamp of vehicles that are outside the scenario
% jihyun- updateTimeMatrix is initial to -1, and it is shape of maxID*maxID
allIDOut = setdiff(all,IDvehicle);
updateTimeMatrix(allIDOut,:) = -1;

% yeomyung
distance = round(sqrt((Xvehicle - Xvehicle').^2)/10) + 1;
distanceReal = sqrt((Xvehicle - Xvehicle').^2);
currentTimeStamp = 0;

% if length(Xvehicle) > 2000
%     inverseOfSampleRate = round(length(Xvehicle)/20);   % ?ëúÎ≥? 30??(border ?†ú?ô∏?ïòÎ©? ?çî ?†ÅÏß?..?ÑàÎ¨? ?ò§?ûòÍ±∏Î†§?Ñú ?ñ¥Ï©? ?àò ?óÜ?ùå..Í∞??ö¥?ç∞ ?ïú ???èÑ ?óÜÍ≤†ÎÉê..)
% else
% inverseOfSampleRate = round(length(Xvehicle)/50);   % ?ëúÎ≥? 50??(border ?†ú?ô∏?ïòÎ©? ?çî ?†ÅÏß?..?ÑàÎ¨? ?ò§?ûòÍ±∏Î†§?Ñú ?ñ¥Ï©? ?àò ?óÜ?ùå)
inverseOfSampleRate = round(length(Xvehicle)/100);   % ?ëúÎ≥? 100??(border ?†ú?ô∏?ïòÎ©? ?çî ?†ÅÏß?..?ÑàÎ¨? ?ò§?ûòÍ±∏Î†§?Ñú ?ñ¥Ï©? ?àò ?óÜ?ùå)
% end

% jihyun - for total logging which log only success one
outFile = fopen(sprintf("./ITTpercent_%d/LOG_Raw%d_VDrange%d_rho%d_MCS%d_%d_success.data", ITTpercent, Raw, rangeForVehicleDensity, rho, MCS, smoothingFactorForITT), 'a');
% jihyun - for total Logging
outFile3 = fopen(sprintf("./ITTpercent_%d/LOG_Raw%d_VDrange%d_rho%d_MCS%d_%d_total.data", ITTpercent, Raw, rangeForVehicleDensity, rho, MCS, smoothingFactorForITT), 'a');

% jihyun - for power Logging
outFile4 = fopen(sprintf("./ITTpercent_%d/LOG_Raw%d_VDrange%d_rho%d_MCS%d_%d_success_power_j.data", ITTpercent, Raw, rangeForVehicleDensity, rho, MCS, smoothingFactorForITT), 'a');
outFile8 = fopen(sprintf("./ITTpercent_%d/LOG_Raw%d_VDrange%d_rho%d_MCS%d_%d_success_power_i.data", ITTpercent, Raw, rangeForVehicleDensity, rho, MCS, smoothingFactorForITT), 'a');
outFile5 = fopen(sprintf("./ITTpercent_%d/LOG_Raw%d_VDrange%d_rho%d_MCS%d_%d_total_power.data", ITTpercent, Raw, rangeForVehicleDensity, rho, MCS, smoothingFactorForITT), 'a');

% jihyun - for rate Logging
outFile6 = fopen(sprintf("./ITTpercent_%d/LOG_Raw%d_VDrange%d_rho%d_MCS%d_%d_success_rate_j.data", ITTpercent, Raw, rangeForVehicleDensity, rho, MCS, smoothingFactorForITT), 'a');
outFile9 = fopen(sprintf("./ITTpercent_%d/LOG_Raw%d_VDrange%d_rho%d_MCS%d_%d_success_rate_i.data", ITTpercent, Raw, rangeForVehicleDensity, rho, MCS, smoothingFactorForITT), 'a');
outFile7 = fopen(sprintf("./ITTpercent_%d/LOG_Raw%d_VDrange%d_rho%d_MCS%d_%d_total_rate.data", ITTpercent, Raw, rangeForVehicleDensity, rho, MCS, smoothingFactorForITT), 'a');


% circle shift
windowForUniqueVehicle = circshift(windowForUniqueVehicle,1,3);
windowForUniqueVehicle(:,:,1) = 0;     % ?ù¥Î≤? ?Ñ¥ Ï¥àÍ∏∞?ôî

for i = 1:Nv
    if mod(i,inverseOfSampleRate)==0     
        % Vehicles inside the awareness range of vehicle with ID i
        IDIn = awarenessID(i,awarenessID(i,:)>0);
        % ID of vehicles that are outside the awareness range of vehicle i
        IDOut = setdiff(all,IDIn);  % Ï∞®Ïßë?ï©
        updateTimeMatrix(IDvehicle(i),IDOut)=-1;
        for j = 1:length(IDIn)
            % jihyun - only looking for i's neighbors ID!!!, neighbors max distance is 470! 
            % If the vehicle is not blocked and if there is no error in
            % reception, update the matrix with the timestamp of the received
            % beacons
            %jihyun - i==Rx, IDIn(j)==Tx
%             if BRid(IDIn(j))>0 && isempty(find(errorMatrix(:,1)==IDvehicle(i) & errorMatrix(:,2)==IDIn(j), 1)) % && i~=j
% Tx(i) is transmit and it is not error - 
%             if BRid(IDvehicle(i))>0 && isempty(find(errorMatrix(:,1)==IDvehicle(i) & errorMatrix(:,2)==IDIn(j), 1)) % && i~=j

            % jihyun- below situation, j is RX vehicle and i is TX vehicle(because currentTimeStamp takes j's time)
            % j is in the road and there is no error between j and i
            % BRid(IDIn(j))>0: vehicle is not blocked
            if BRid(IDIn(j))>0 && isempty(find(errorMatrix(:,1)==IDvehicle(i) & errorMatrix(:,2)==IDIn(j), 1)) % && i~=j
%             if BRid(i)>0 && isempty(find(errorMatrix(:,1)==IDvehicle(i) & errorMatrix(:,2)==IDIn(j), 1)) % && i~=j
%                 if mod(i,inverseOfSampleRate)==0    % ?ïΩ 50?? or 30??Îß? Ï∂îÏ∂ú?ïò?ó¨Í≥ÑÏÇ∞
                    % Store previous timestamp
                    previousTimeStamp = updateTimeMatrix(IDvehicle(i),IDIn(j));
                    % Compute current timestamp (at Rx(j))
                    currentTimeStamp = elapsedTime-(0.1/NbeaconsT)*(NbeaconsT-BRidT(IDIn(j)));
%jihyun - this is for change i to Tx and j to Rx
%                     currentTimeStamp = elapsedTime-(0.1/NbeaconsT)*(NbeaconsT-BRidT(i));
                    % If there was a previous timestamp
                    if previousTimeStamp > 0.1
                        % Compute update delay, considering the subframe used for transmission (s)
                        updateDelay = currentTimeStamp-previousTimeStamp;    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                        % yeomyung
                        %                 if elapsedTime >= 0.3       %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% perturbation period????
                        %                     % distance = round(sqrt((Xvehicle - Xvehicle').^2+(Yvehicle - Yvehicle').^2));
                        %                     distanceOfTwo = distance(i,awarenessID(i,j));        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                        %                     if distanceOfTwo == 0
                        %                         distanceOfTwo = 1;
                        %                     end
                        distanceOfTwo = distance(i,IDIn(j));        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

                        % if elapsedTime >= 30.0
                        % distanceOfTwo = distance(i,awarenessID(i,j));        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                         if distanceOfTwo <= 60  &&  Xvehicle(IDIn(j)) >= 700.0 && Xvehicle(IDIn(j)) <= 1100.0
                        if distanceOfTwo <= 60  &&  Xvehicle(i) >= 700.0 && Xvehicle(i) <= 1100.0
                            % border ?†ú?ô∏ - Rx ?Ö∏?ìúÍ∞? border ?ïà?óê ?ûà?ùÑ ?ïåÎß? Í≥ÑÏÇ∞
                            % if Xvehicle(i) >= 600.0 && Xvehicle(i) <= 1200.0
                            % SLT?äî pairÎ≥ÑÎ°ú ?ã§ count ?ï¥?ïº..? - ?ù¥Í±? Í≥ÑÏÇ∞?ï¥?Ñú ?ôï?ù∏?ï¥?ïº.. -> mainLTEV2V?èÑ ?àò?†ï?ï¥?ïº ?ï¥. SLT 40Ï¥àÎ°ú Í≥ÑÏÇ∞?ñà?ûñ?ïÑ..(0.3 / 40) ?ù¥Í±? Í≥±Ìñà?ûñ?ïÑ..?àò?†ï?ï¥?Ñú Í≥ÑÏÇ∞?ï¥Î¥?
                            % Î°úÍ∑∏?óê tx, rx, Í∞ÅÍ∞Å?ùò ?úÑÏπ?(Ï¢åÌëú)?èÑ ?ì§?ñ¥Í∞??ïº ?ïú?ã§
                            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                            
                            % Information Age
                            sumOfUpdateDelayForDistance(distanceOfTwo)...
                                = sumOfUpdateDelayForDistance(distanceOfTwo) + updateDelay;
                            numOfUpdateForDistance(distanceOfTwo) = numOfUpdateForDistance(distanceOfTwo) + 1;
                            % PRR
                            numOfRx(distanceOfTwo) = numOfRx(distanceOfTwo) + 1;
                            numOfTx(distanceOfTwo) = numOfTx(distanceOfTwo) + 1;
                            
                            %for rate control Tx vehicle
                            if IDIn(j)>powerVehicle
                                RatesumOfUpdateDelayForDistance(distanceOfTwo)...
                                    = RatesumOfUpdateDelayForDistance(distanceOfTwo) + updateDelay;
                                RatenumOfUpdateForDistance(distanceOfTwo) = RatenumOfUpdateForDistance(distanceOfTwo) + 1;
                                % PRR
                                RatenumOfRx(distanceOfTwo) = RatenumOfRx(distanceOfTwo) + 1;
                                RatenumOfTx(distanceOfTwo) = RatenumOfTx(distanceOfTwo) + 1;
                                %for power control Tx vehicle
                            else
                                PowersumOfUpdateDelayForDistance(distanceOfTwo)...
                                    = PowersumOfUpdateDelayForDistance(distanceOfTwo) + updateDelay;
                                PowernumOfUpdateForDistance(distanceOfTwo) = PowernumOfUpdateForDistance(distanceOfTwo) + 1;
                                % PRR
                                PowernumOfRx(distanceOfTwo) = PowernumOfRx(distanceOfTwo) + 1;
                                PowernumOfTx(distanceOfTwo) = PowernumOfTx(distanceOfTwo) + 1;
                            end
                            
                            % for tmp which using for Tx by i
                            if i>powerVehicle
                                tmpRatesumOfUpdateDelayForDistance(distanceOfTwo)...
                                    = tmpRatesumOfUpdateDelayForDistance(distanceOfTwo) + updateDelay;
                                tmpRatenumOfUpdateForDistance(distanceOfTwo) = tmpRatenumOfUpdateForDistance(distanceOfTwo) + 1;
                                % PRR
                                tmpRatenumOfRx(distanceOfTwo) = tmpRatenumOfRx(distanceOfTwo) + 1;
                                tmpRatenumOfTx(distanceOfTwo) = tmpRatenumOfTx(distanceOfTwo) + 1;
                                %for power control Tx vehicle
                            else
                                tmpPowersumOfUpdateDelayForDistance(distanceOfTwo)...
                                    = tmpPowersumOfUpdateDelayForDistance(distanceOfTwo) + updateDelay;
                                tmpPowernumOfUpdateForDistance(distanceOfTwo) = tmpPowernumOfUpdateForDistance(distanceOfTwo) + 1;
                                % PRR
                                tmpPowernumOfRx(distanceOfTwo) = tmpPowernumOfRx(distanceOfTwo) + 1;
                                tmpPowernumOfTx(distanceOfTwo) = tmpPowernumOfTx(distanceOfTwo) + 1;
                            end
                            % SLT
                            % ?ñ¥Ï∞®Ìîº ?ã§ ?ï©Ï≥êÏÑú ?èâÍ∑†Ïù¥?ãàÍπ?.. numOfRxÎ°? ?ï¥Î≤ÑÎ¶¨?üà.. Î∂ÑÎ™®Îß? Îπ†Ï??äî Í≤©Ïù¥Ïß?...
                            % Í∑∏Îü¨Î©? ?ïà ?ê† ?àò?èÑ.. Rx Í∏∞Ï??úºÎ°? all node-pair ?óê ???ïò?ó¨ ?èâÍ∑?..
                            % reference ?ÖºÎ¨∏Ïóê?Ñú?äî 1Ï¥àÏóê 1.9KB -> 0.1Ï¥àÏóê 190Î∞îÏù¥?ä∏ Î∞õÏúºÎ©? 100% ?Ñ±Í≥µÏù∏ Í±∞Ï?..
                            % ?Ç¥ Í≤ΩÏö∞?óê?äî 3.0Kbps Í∞? ?Çò???ïº 100% ?Ñ±Í≥µÏù∏ Í≤ÉÏù¥?ã§..
                            % DCC ?†Å?ö© ?ïà?ï¥?èÑ Î©??àòÎ°? ?ÇÆ?ã§ - ?†Ñ?ã¨Î•†Ïù¥ Î∞òÏòÅ?êú Í≤ÉÏù¥Í≥?
                            % DCC ?†Å?ö© ?õÑ Í∞?ÍπåÏù¥?óê?Ñú?èÑ ?ÇÆ?? Í±? Î≥¥Î©¥ - ITT?èÑ ?ï®Íª? Î∞òÏòÅ?êú Í≤ÉÏù¥?ã§.
                            
                            % end
                            % if printLOG
                            % LOG (success) - 1)Rx  2)Tx  3) Rx locate 4)
                            % Tx locate 5)Rx-Tx distance
                            %   6)currentTimestamp 7)transmission
                            % success: 1, fail: 0 8)updateDelay
                            
                            % Í∏∞Ï°¥?óê ?ûà?çòÍ≤?
                            fprintf(outFile, '%d\t%d\t%f\t%f\t%d\t%f\t1\t%f\n', i, IDIn(j),  Xvehicle(i), Xvehicle(IDIn(j)), (distanceOfTwo - 1)*10, currentTimeStamp, updateDelay);
                            fprintf(outFile3, '%d\t%d\t%f\t%f\t%d\t%f\t1\t%f\n', i, IDIn(j),  Xvehicle(i), Xvehicle(IDIn(j)), (distanceOfTwo - 1)*10, currentTimeStamp, updateDelay);
                            
                            %Tx is j!
                            if IDIn(j) > powerVehicle
                            % this for rate control(6-success, 7-total)
                            if updateDelay>=0.1
                                fprintf(outFile6, '%d\t%d\t%f\t%f\t%d\t%f\t1\t%f\n', i, IDIn(j),  Xvehicle(i), Xvehicle(IDIn(j)), (distanceOfTwo - 1)*10, currentTimeStamp, updateDelay);
                            end
                                fprintf(outFile7, '%d\t%d\t%f\t%f\t%d\t%f\t1\t%f\n', i, IDIn(j),  Xvehicle(i), Xvehicle(IDIn(j)), (distanceOfTwo - 1)*10, currentTimeStamp, updateDelay); 
                            else
                                % this is for power control(4-success, 5-total)
                                if updateDelay>=0.1
                                fprintf(outFile4, '%d\t%d\t%f\t%f\t%d\t%f\t1\t%f\n', i, IDIn(j),  Xvehicle(i), Xvehicle(IDIn(j)), (distanceOfTwo - 1)*10, currentTimeStamp, updateDelay);
                                end
                                fprintf(outFile5, '%d\t%d\t%f\t%f\t%d\t%f\t1\t%f\n', i, IDIn(j),  Xvehicle(i), Xvehicle(IDIn(j)), (distanceOfTwo - 1)*10, currentTimeStamp, updateDelay); 
                            end            
                            
                            %for tmp which is using to discrete Tx is i
                            if i > powerVehicle
                            % this for rate control
                                fprintf(outFile9, '%d\t%d\t%f\t%f\t%d\t%f\t1\t%f\n', i, IDIn(j),  Xvehicle(i), Xvehicle(IDIn(j)), (distanceOfTwo - 1)*10, currentTimeStamp, updateDelay); 
                            else
                                % this is for power control
                                fprintf(outFile8, '%d\t%d\t%f\t%f\t%d\t%f\t1\t%f\n', i, IDIn(j),  Xvehicle(i), Xvehicle(IDIn(j)), (distanceOfTwo - 1)*10, currentTimeStamp, updateDelay); 
                            end
                            
                            
                            
                            % end
                        end

                    end
                    % Update updateTimeMatrix with the current timestamp
                    updateTimeMatrix(IDvehicle(i),IDIn(j)) = currentTimeStamp;

%             elseif mod(i,inverseOfSampleRate)==0  && ~isempty(find(errorMatrix(:,1)==IDvehicle(i) & errorMatrix(:,2)==IDIn(j), 1))
            elseif ~isempty(find(errorMatrix(:,1)==IDvehicle(i) & errorMatrix(:,2)==IDIn(j), 1))

                % If there was a previous timestamp - Î¨∏Ï†ú ?ûà?ùÑ Ïß??èÑ Î™®Î¶Ñ%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                % if previousTimeStamp > 0.1
                %                 distanceOfTwo = distance(i,awarenessID(i,j));
                distanceOfTwo = distance(i,IDIn(j));        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                
%                 if distanceOfTwo <= 60 && Xvehicle(IDIn(j)) >= 700.0 && Xvehicle(IDIn(j)) <= 1100.0
                if distanceOfTwo <= 60 && Xvehicle(i) >= 700.0 && Xvehicle(i) <= 1100.0
                    %                     if printLOG
                    % LOG (failure) - 1)Rx  2)Tx  3)Rx-Tx Í∞? distance  4)currentTimestamp  5)transmission success: 1, fail: 0
                    fprintf(outFile3, '%d\t%d\t%f\t%f\t%d\t%f\t0\t0\n', i, awarenessID(i,j), Xvehicle(i), Xvehicle(awarenessID(i,j)), (distanceOfTwo - 1)*10, updateTimeMatrix(IDvehicle(i),IDIn(j)));
                    if IDIn(j)<= powerVehicle
                        % for power total log
                        fprintf(outFile5, '%d\t%d\t%f\t%f\t%d\t%f\t0\t0\n', i, awarenessID(i,j), Xvehicle(i), Xvehicle(awarenessID(i,j)), (distanceOfTwo - 1)*10, updateTimeMatrix(IDvehicle(i),IDIn(j)));
                    else
                        % for rate total log
                        fprintf(outFile7, '%d\t%d\t%f\t%f\t%d\t%f\t0\t0\n', i, awarenessID(i,j), Xvehicle(i), Xvehicle(awarenessID(i,j)), (distanceOfTwo - 1)*10, updateTimeMatrix(IDvehicle(i),IDIn(j)));
                    end
                    %                     if awarenessID(i,j)> endpoint
                    %                         % rate control ?ïú ?ï†?ì§ Î°úÍ∑∏ ?Ç®Íπ?
                    %                         fprintf(outFile, '%d\t%d\t%f\t%f\t%d\t%f\t0\t0\n', i, awarenessID(i,j), Xvehicle(i), Xvehicle(awarenessID(i,j)), (distanceOfTwo - 1)*10, updateTimeMatrix(IDvehicle(i),IDIn(j)));
                    %                     else                        % rate control ?ïú ?ï†?ì§ Î°úÍ∑∏ ?Ç®Íπ?
                    %                         % power control ?ïú ?ï†?ì§ Î°úÍ∑∏ ?Ç®Íπ?
                    %                         fprintf(outFile2, '%d\t%d\t%f\t%f\t%d\t%f\t0\t0\n', i, awarenessID(i,j), Xvehicle(i), Xvehicle(awarenessID(i,j)), (distanceOfTwo - 1)*10, updateTimeMatrix(IDvehicle(i),IDIn(j)));
                    %                     end
                    
                    %                     end
                    
                    
                    numOfTx(distanceOfTwo) = numOfTx(distanceOfTwo) + 1; %
                    if IDIn(j)>powerVehicle
                        %for rate control vehicle
                        RatenumOfTx(distanceOfTwo) = RatenumOfTx(distanceOfTwo) + 1; %
                    else
                        %for power control vehicle
                        PowernumOfTx(distanceOfTwo) = PowernumOfTx(distanceOfTwo) + 1; %
                    end
                    
                    % for tmp
                     if i>powerVehicle
                        %for rate control vehicle
                        tmpRatenumOfTx(distanceOfTwo) = tmpRatenumOfTx(distanceOfTwo) + 1; %
                    else
                        %for power control vehicle
                        tmpPowernumOfTx(distanceOfTwo) = tmpPowernumOfTx(distanceOfTwo) + 1; %
                    end
                    %             end
                    % end
                end
                % end
            end
        end
    end
end

% ?åå?ùº ?ã´?ïÑ
% if printLOG
fclose(outFile);
fclose(outFile3);
fclose(outFile4);
fclose(outFile5);
fclose(outFile6);
fclose(outFile7);


% end

end
