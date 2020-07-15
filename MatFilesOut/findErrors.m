function [errorMatrix, NerrorHist, BLERHist] = findErrors(IDvehicle,awarenessID,awarenessSINR,awarenessBRid,distance,gammaMin, elapsedTime, NerrorHist, BLERHist)
% Detect wrongly decoded beacons and create Error Matrix
% [ID RX, ID TX, BRid, distance]

Nv = length(IDvehicle);                   % Total number of vehicles
errorMatrix = zeros(Nv*Nv-1,4);           % Initialize collision matrix
Nerrors = 0;                              % Initialize number of errors

successMatrix = zeros(length(IDvehicle), length(IDvehicle)-1);

for i = 1:Nv
    index = find(awarenessID(i,:));
    if ~isempty(index)
        for j = 1:length(index)
            % BLER
            [noErr, randomval, BLER] = CheckBLER(awarenessSINR(i,j));
            %BLER = CheckBLER(awarenessSINR(i,index(j)));
            BLERHist(round(elapsedTime/0.1) * Nv + j, 1) = BLER;
            BLERHist(round(elapsedTime/0.1) * Nv + j, 2) = awarenessSINR(i,j);
            
            % If received beacon SINR is lower than the threshold
            % or error result from CheckBLER
%             if (awarenessBRid(i,index(j))>0 && awarenessSINR(i,index(j))<gammaMin) || (awarenessBRid(i,index(j))>0  && ~noErr)  % ?씠嫄? ?븘?땲?빞. 450m?뿉?꽌 瑗щ━媛? ?걡湲곗옏?븘.. ?걡湲곕뒗 寃? 留욎?..MCS7?씠 留? 1km源뚯? 媛??옿..

            if awarenessBRid(i,index(j))>0 && ~noErr            % BLER 留? ?뀒?뒪?듃

%            if (awarenessBRid(i,index(j))>0 && awarenessSINR(i,index(j))<gammaMin)           % BLER 鍮쇨퀬 ?뀒?뒪?듃
            
                Nerrors = Nerrors + 1;
                errorMatrix(Nerrors,1) = IDvehicle(i);
                errorMatrix(Nerrors,2) = awarenessID(i,index(j));
                errorMatrix(Nerrors,3) = awarenessBRid(i,index(j));
                errorMatrix(Nerrors,4) = distance(i,IDvehicle==awarenessID(i,index(j)));
            
            
            else
                % yeomyung
                % check successful BRid
                if awarenessBRid(i,index(j))>0
                    successMatrix(i, index(j)) = awarenessBRid(i,index(j));
                end
            end      
        end
    end 
end

% yeomyung - 아래 에러 수정 (시뮬레이터 오류)
        % 그리고 상록이도 LTEv2vsim 에러를 찾은 것이 있었는데
        % 수신여부 판정에서 어떤 threshold 이상이면 수신, 아니면 못 받은 것으로 하는 logic이 있었는데
        % 패킷 2개가 같은 RB에서 수신이 되었을 때 둘 다 수신이라는 말도 안되는 결정을 내리는 걸
        % 찾아냈었다.
        % 아...상록이 코드로 할 걸 그랬나....
% 위에서 저장해 둔 successMatrix 에서 중복을 찾아 가장 높은 SINR 를 갖는 것 외에 중복되는 수신은 에러 처리 - 이게 아니라 collision이지..대체 이런 어이없는 생각은 어떻게 한 거냐..
% matlab API 중 unique() 함수 도움말 예제를 참고하였음
for i=1:Nv
    [C, ia, ic] = unique(successMatrix(i,:), 'sorted');
    a_counts = accumarray(ic,1);
    value_counts = [C', a_counts];
    for j=1:length(value_counts(:,1))
        if  value_counts(j,1) > 0 && value_counts(j,2) > 1    % 以묐났 諛쒓껄!!
            DuplicatedBRid = value_counts(j,1);
            DuplicatedIndex = find(successMatrix(i,:) == DuplicatedBRid);
            for k=1:length(DuplicatedIndex)
                Nerrors = Nerrors + 1;
                errorMatrix(Nerrors,1) = IDvehicle(i);
                errorMatrix(Nerrors,2) = awarenessID(i,DuplicatedIndex(k));
                errorMatrix(Nerrors,3) = DuplicatedBRid;
                errorMatrix(Nerrors,4) = distance(i,awarenessID(i,DuplicatedIndex(k)));
            end
            
            
            % DuplicatedAwarenessID = awarenessID(i,DuplicatedIndex);
%             temp = zeros(length(DuplicatedIndex));
%             temp = awarenessSINR(i,DuplicatedIndex);
% 
%             [sortedSINR, sortedIndex] = sort(temp);
%             % SINR ?씠 媛??옣 ?겙 Vehicle ?쇅?뿉?뒗 ?뿉?윭泥섎━
%             for k=1:length(DuplicatedIndex) - 1
%                 Nerrors = Nerrors + 1;
%                 errorMatrix(Nerrors,1) = IDvehicle(i);
%                 errorMatrix(Nerrors,2) = DuplicatedIndex(sortedIndex(k));
%                 errorMatrix(Nerrors,3) = DuplicatedBRid;
%                 errorMatrix(Nerrors,4) = distance(i,IDvehicle==DuplicatedIndex(sortedIndex(k)));
%             end
            
        end
    end
end

NerrorHist(round(elapsedTime/0.1), 1) = Nerrors;

delIndex = errorMatrix(:,1)==0;
errorMatrix(delIndex,:) = [];

end
