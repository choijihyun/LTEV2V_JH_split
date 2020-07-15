function [updateTimeMatrix,updateDelayCounter, sumOfUpdateDelayForDistance, numOfUpdateForDistance, numOfTx, numOfRx, windowForUniqueVehicle]...
    = countUpdateDelay(IDvehicle,BRid,NbeaconsT,awarenessID,errorMatrix,elapsedTime,updateTimeMatrix,updateDelayCounter,delayResolution, ...
    sumOfUpdateDelayForDistance, numOfUpdateForDistance, numOfTx, numOfRx, Xvehicle, Yvehicle, Raw, rangeForVehicleDensity, rho, MCS, smoothingFactorForITT, printLOG, ITTpercent, windowForUniqueVehicle)
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
%     inverseOfSampleRate = round(length(Xvehicle)/30);   % ?๋ณ? 30??(border ? ?ธ?๋ฉ? ? ? ์ง?..?๋ฌ? ?ค?๊ฑธ๋ ค? ?ด์ฉ? ? ??..๊ฐ??ด?ฐ ? ??? ?๊ฒ ๋..)
% else
inverseOfSampleRate = round(length(Xvehicle)/50);   % ?๋ณ? 50??(border ? ?ธ?๋ฉ? ? ? ์ง?..?๋ฌ? ?ค?๊ฑธ๋ ค? ?ด์ฉ? ? ??)
% inverseOfSampleRate = round(length(Xvehicle)/100);   % ?๋ณ? 100??(border ? ?ธ?๋ฉ? ? ? ์ง?..?๋ฌ? ?ค?๊ฑธ๋ ค? ?ด์ฉ? ? ??)
% end

% jihyun - for rate control vehicles
outFile = fopen(sprintf("./ITTpercent_%d/LOG_Raw%d_VDrange%d_rho%d_MCS%d_%d_success.data", ITTpercent, Raw, rangeForVehicleDensity, rho, MCS, smoothingFactorForITT), 'a');
% jihyun - for power control vehicles
%๋ถ์?ด?... ????๋ก? ?ค ???ฅ??๊ฑ?!
outFile3 = fopen(sprintf("./ITTpercent_%d/LOG_Raw%d_VDrange%d_rho%d_MCS%d_%d_total.data", ITTpercent, Raw, rangeForVehicleDensity, rho, MCS, smoothingFactorForITT), 'a');


% circle shift
windowForUniqueVehicle = circshift(windowForUniqueVehicle,1,3);
windowForUniqueVehicle(:,:,1) = 0;     % ?ด๋ฒ? ?ด ์ด๊ธฐ?

for i = 1:Nv
    if mod(i,inverseOfSampleRate)==0     
        % Vehicles inside the awareness range of vehicle with ID i
        IDIn = awarenessID(i,awarenessID(i,:)>0);
        % ID of vehicles that are outside the awareness range of vehicle i
        IDOut = setdiff(all,IDIn);  % ์ฐจ์ง?ฉ
        updateTimeMatrix(IDvehicle(i),IDOut)=-1;
        for j = 1:length(IDIn)
            % jihyun - only looking for i's neighbors ID!!!, neighbors max distance is 470! 
            % If the vehicle is not blocked and if there is no error in
            % reception, update the matrix with the timestamp of the received
            % beacons
            %jihyun - j==Rx, i==Tx
%             if BRid(IDIn(j))>0 && isempty(find(errorMatrix(:,1)==IDvehicle(i) & errorMatrix(:,2)==IDIn(j), 1)) % && i~=j
% Tx(i) is transmit and it is not error - 
%             if BRid(IDvehicle(i))>0 && isempty(find(errorMatrix(:,1)==IDvehicle(i) & errorMatrix(:,2)==IDIn(j), 1)) % && i~=j

            % jihyun- below situation, j is RX vehicle and i is TX vehicle(because currentTimeStamp takes j's time)
            % j is in the road and there is no error between j and i
            % BRid(IDIn(j))>0: vehicle is not blocked
            if BRid(IDIn(j))>0 && isempty(find(errorMatrix(:,1)==IDvehicle(i) & errorMatrix(:,2)==IDIn(j), 1)) % && i~=j
%                 if mod(i,inverseOfSampleRate)==0    % ?ฝ 50?? or 30??๋ง? ์ถ์ถ??ฌ๊ณ์ฐ
                    % Store previous timestamp
                    previousTimeStamp = updateTimeMatrix(IDvehicle(i),IDIn(j));
                    % Compute current timestamp (at Rx(j))
                    currentTimeStamp = elapsedTime-(0.1/NbeaconsT)*(NbeaconsT-BRidT(IDIn(j)));
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
                        aaaaa = IDIn(j);
                        bbbb = awarenessID(i,j);
                        % if elapsedTime >= 30.0
                        % distanceOfTwo = distance(i,awarenessID(i,j));        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                        if distanceOfTwo <= 60  &&  Xvehicle(IDIn(j)) >= 700.0 && Xvehicle(IDIn(j)) <= 1100.0
                            % border ? ?ธ - Rx ?ธ?๊ฐ? border ?? ?? ?๋ง? ๊ณ์ฐ
                            % if Xvehicle(i) >= 600.0 && Xvehicle(i) <= 1200.0
                            % SLT? pair๋ณ๋ก ?ค count ?ด?ผ..? - ?ด๊ฑ? ๊ณ์ฐ?ด? ??ธ?ด?ผ.. -> mainLTEV2V? ?? ?ด?ผ ?ด. SLT 40์ด๋ก ๊ณ์ฐ???..(0.3 / 40) ?ด๊ฑ? ๊ณฑํ??..?? ?ด? ๊ณ์ฐ?ด๋ด?
                            % ๋ก๊ทธ? tx, rx, ๊ฐ๊ฐ? ?์น?(์ขํ)? ?ค?ด๊ฐ??ผ ??ค
                            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                            
                            % Information Age
                            sumOfUpdateDelayForDistance(distanceOfTwo)...
                                = sumOfUpdateDelayForDistance(distanceOfTwo) + updateDelay;
                            numOfUpdateForDistance(distanceOfTwo) = numOfUpdateForDistance(distanceOfTwo) + 1;
                            % PRR
                            numOfRx(distanceOfTwo) = numOfRx(distanceOfTwo) + 1;
                            numOfTx(distanceOfTwo) = numOfTx(distanceOfTwo) + 1;
                            
                            % SLT
                            % ?ด์ฐจํผ ?ค ?ฉ์ณ์ ?๊ท ์ด?๊น?.. numOfRx๋ก? ?ด๋ฒ๋ฆฌ?.. ๋ถ๋ชจ๋ง? ๋น ์?? ๊ฒฉ์ด์ง?...
                            % ๊ทธ๋ฌ๋ฉ? ? ?  ??.. Rx ๊ธฐ์??ผ๋ก? all node-pair ? ????ฌ ?๊ท?..
                            % reference ?ผ๋ฌธ์?? 1์ด์ 1.9KB -> 0.1์ด์ 190๋ฐ์ด?ธ ๋ฐ์ผ๋ฉ? 100% ?ฑ๊ณต์ธ ๊ฑฐ์?..
                            % ?ด ๊ฒฝ์ฐ?? 3.0Kbps ๊ฐ? ????ผ 100% ?ฑ๊ณต์ธ ๊ฒ์ด?ค..
                            % DCC ? ?ฉ ??ด? ๋ฉ??๋ก? ?ฎ?ค - ? ?ฌ๋ฅ ์ด ๋ฐ์? ๊ฒ์ด๊ณ?
                            % DCC ? ?ฉ ? ๊ฐ?๊น์ด??? ?ฎ?? ๊ฑ? ๋ณด๋ฉด - ITT? ?จ๊ป? ๋ฐ์? ๊ฒ์ด?ค.
                            
                            % end
                            % if printLOG
                            % LOG (success) - 1)Rx  2)Tx  3) Rx? ?์น? 4) Tx? ?์น?  5)Rx-Tx ๊ฐ?
                            % distance  6)currentTimestamp 7)transmission
                            % success: 1, fail: 0 8)updateDelay
                            
                            % ๊ธฐ์กด? ??๊ฒ?
                            fprintf(outFile, '%d\t%d\t%f\t%f\t%d\t%f\t1\t%f\n', i, IDIn(j),  Xvehicle(i), Xvehicle(IDIn(j)), (distanceOfTwo - 1)*10, currentTimeStamp, updateDelay);
                            fprintf(outFile3, '%d\t%d\t%f\t%f\t%d\t%f\t1\t%f\n', i, IDIn(j),  Xvehicle(i), Xvehicle(IDIn(j)), (distanceOfTwo - 1)*10, currentTimeStamp, updateDelay);
                            
                            % end
                        end
                        % end
                        
                        
                        
                        %                     % Check if the update delay is larger than the maximum delay value stored in the array
                        %                     if updateDelay>=delayMax
                        %                         % Increment last counter
                        %                         updateDelayCounter(end) = updateDelayCounter(end) + 1;
                        %                     else
                        %                         % Increment counter corresponding to the current delay
                        %                         updateDelayCounter(ceil(updateDelay/delayResolution)) = ...
                        %                             updateDelayCounter(ceil(updateDelay/delayResolution)) + 1;
                        %                     end
                        %                 end
                    end
                    % Update updateTimeMatrix with the current timestamp
                    updateTimeMatrix(IDvehicle(i),IDIn(j)) = currentTimeStamp;
                    
                    % yeomyung
                    %             if distance(IDvehicle(i),IDIn(j))<100
                    %                 neighborCountBasedBSM(i) = neighborCountBasedBSM(i) + 1;
                    %             end
%                 end
                
                % error? IA ๊ณ์ฐ?๋ง? ?? - 30์ด? ?ด?? ??๋ง?
%             elseif mod(i,inverseOfSampleRate)==0  && ~isempty(find(errorMatrix(:,1)==IDvehicle(i) & errorMatrix(:,2)==IDIn(j), 1))
            elseif ~isempty(find(errorMatrix(:,1)==IDvehicle(i) & errorMatrix(:,2)==IDIn(j), 1))

                % If there was a previous timestamp - ๋ฌธ์  ?? ์ง?? ๋ชจ๋ฆ%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                % if previousTimeStamp > 0.1
                %                 distanceOfTwo = distance(i,awarenessID(i,j));
                distanceOfTwo = distance(i,IDIn(j));        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                
                if distanceOfTwo <= 60 && Xvehicle(IDIn(j)) >= 700.0 && Xvehicle(IDIn(j)) <= 1100.0
                    %                     if printLOG
                    % LOG (failure) - 1)Rx  2)Tx  3)Rx-Tx ๊ฐ? distance  4)currentTimestamp  5)transmission success: 1, fail: 0
                    fprintf(outFile3, '%d\t%d\t%f\t%f\t%d\t%f\t0\t0\n', i, awarenessID(i,j), Xvehicle(i), Xvehicle(awarenessID(i,j)), (distanceOfTwo - 1)*10, updateTimeMatrix(IDvehicle(i),IDIn(j)));
                    %                     if awarenessID(i,j)> endpoint
                    %                         % rate control ? ? ?ค ๋ก๊ทธ ?จ๊น?
                    %                         fprintf(outFile, '%d\t%d\t%f\t%f\t%d\t%f\t0\t0\n', i, awarenessID(i,j), Xvehicle(i), Xvehicle(awarenessID(i,j)), (distanceOfTwo - 1)*10, updateTimeMatrix(IDvehicle(i),IDIn(j)));
                    %                     else                        % rate control ? ? ?ค ๋ก๊ทธ ?จ๊น?
                    %                         % power control ? ? ?ค ๋ก๊ทธ ?จ๊น?
                    %                         fprintf(outFile2, '%d\t%d\t%f\t%f\t%d\t%f\t0\t0\n', i, awarenessID(i,j), Xvehicle(i), Xvehicle(awarenessID(i,j)), (distanceOfTwo - 1)*10, updateTimeMatrix(IDvehicle(i),IDIn(j)));
                    %                     end
                    
                    %                     end
                    
                    
                    numOfTx(distanceOfTwo) = numOfTx(distanceOfTwo) + 1; % ??ฌ ???ผ๋ฉ? Tx๋ง? count
                    %             end
                    % end
                end
                % end
            end
        end
    end
end

% ??ผ ?ซ?
% if printLOG
fclose(outFile);
fclose(outFile3);


% end

end
