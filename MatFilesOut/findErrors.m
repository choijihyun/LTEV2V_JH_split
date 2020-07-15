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
%             if (awarenessBRid(i,index(j))>0 && awarenessSINR(i,index(j))<gammaMin) || (awarenessBRid(i,index(j))>0  && ~noErr)  % ?���? ?��?��?��. 450m?��?�� 꼬리�? ?��기잖?��.. ?��기는 �? 맞�?..MCS7?�� �? 1km까�? �??��..

            if awarenessBRid(i,index(j))>0 && ~noErr            % BLER �? ?��?��?��

%            if (awarenessBRid(i,index(j))>0 && awarenessSINR(i,index(j))<gammaMin)           % BLER 빼고 ?��?��?��
            
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

% yeomyung - �Ʒ� ���� ���� (�ùķ����� ����)
        % �׸��� ����̵� LTEv2vsim ������ ã�� ���� �־��µ�
        % ���ſ��� �������� � threshold �̻��̸� ����, �ƴϸ� �� ���� ������ �ϴ� logic�� �־��µ�
        % ��Ŷ 2���� ���� RB���� ������ �Ǿ��� �� �� �� �����̶�� ���� �ȵǴ� ������ ������ ��
        % ã�Ƴ¾���.
        % ��...����� �ڵ�� �� �� �׷���....
% ������ ������ �� successMatrix ���� �ߺ��� ã�� ���� ���� SINR �� ���� �� �ܿ� �ߺ��Ǵ� ������ ���� ó�� - �̰� �ƴ϶� collision����..��ü �̷� ���̾��� ������ ��� �� �ų�..
% matlab API �� unique() �Լ� ���� ������ �����Ͽ���
for i=1:Nv
    [C, ia, ic] = unique(successMatrix(i,:), 'sorted');
    a_counts = accumarray(ic,1);
    value_counts = [C', a_counts];
    for j=1:length(value_counts(:,1))
        if  value_counts(j,1) > 0 && value_counts(j,2) > 1    % 중복 발견!!
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
%             % SINR ?�� �??�� ?�� Vehicle ?��?��?�� ?��?��처리
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
