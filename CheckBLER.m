function [noErr, randomval, BLER] = CheckBLER(SINR)
% function BLER = CheckBLER(SINR)

    SINR_step = 2;

%     if MCS == 7
        base_SINR = [-2     0       2       4       6       8       10      12];
        % yongseok
        % base_BLER = [0.99    0.8    0.53    0.27    0.088   0.025   0.008   0.001];
        % sangrok
        % 300	QPSK	0.7	280
        base_BLER = [1      1       0.95	0.83	0.53	0.22	0.077       0.03];%        0.015       1.00E-03    1.00E-03	1.00E-04];
        % 300	QPSK	0.7	30
        % BLER(4,:) = [1      0.94	0.8     0.55	0.31	0.13	0.042       0.012       0.007       1.00E-03	1.00E-03	1.00E-04];
        
        
        
        if SINR <= -2
            BLER = 1;
        elseif SINR > 10
            if SINR >= 30
                BLER = 1e-5;
            elseif SINR >= 20
                BLER = 1e-4;
            else          
                BLER = 1e-3;
            end
        else
            base_idx = floor(SINR / 2 + 1);
            weight = (SINR - base_SINR(base_idx)) / SINR_step;
            BLER = base_BLER(base_idx) * (1 - weight) + base_BLER(base_idx + 1) * weight;
        end

        randomval = rand();
        if randomval < BLER
            noErr = false;
        else
            noErr = true;
        end
            

%% BLER 

% 300	QPSK	0.5	280
% 300	QPSK	0.5	30
% 300	QPSK	0.7	280
% 300	QPSK	0.7	30
% 190	QPSK	0.5	280
% 190	QPSK	0.5	30
% 190	QPSK	0.7	280
% 190	QPSK	0.7	30
% 
% SINR_base = [-2     0       2       4       6       8       10          12          14          16          18          20];
% BLER(1,:) = [1      0.95	0.6     0.2     0.04	0.008	1.00E-03	1.00E-03	1.00E-03	1.00E-03	1.00E-04	1.00E-04];
% BLER(2,:) = [0.95	0.80	0.45	0.2     0.065	0.015	0.002       1.00E-03	1.00E-03	1.00E-03	1.00E-04	1.00E-05];
% BLER(3,:) = [1      1       0.95	0.83	0.53	0.22	0.077       0.03        0.015       1.00E-03    1.00E-03	1.00E-04];
% BLER(4,:) = [1      0.94	0.8     0.55	0.31	0.13	0.042       0.012       0.007       1.00E-03	1.00E-03	1.00E-04];
% BLER(5,:) = [1      0.9     0.7     0.3     0.09	0.02	0.002       1.00E-03	1.00E-03	1.00E-03	1.00E-03	1.00E-04];
% BLER(6,:) = [0.95	0.8     0.65	0.3     0.1     0.03	0.01        1.00E-03	1.00E-03	1.00E-03	1.00E-03	1.00E-04];
% BLER(7,:) = [1      1       0.9     0.7     0.4     0.13	0.045       0.017       0.007       1.00E-03	1.00E-03	1.00E-04];
% BLER(8,:) = [1      0.95	0.75	0.5     0.29	0.11	0.04        0.01        0.0007      1.00E-03    1.00E-03	1.00E-04];
% 
% ret = 0;
% for i=1:1:12
%     if (SINR_base(i) < SINR)
%         ret = i;
%         continue;
%     end
%     
%     if (SINR < SINR_base(i))
%         break;
%     end
% end
% 
% noErr = 1;
% if (ret == 0)
%     noErr = 0;
% else
%     randomval = rand();
%     if (randomval < BLER(3,ret))
% %         if (distance(iv, jv) < 100)
% %             msg = sprintf("BLER %d, [%3d %3d] [%3d %3d] idx %d, SINR: %f, randval: %.5f / base: %.5f / %.3f\n",...
% %                 caller, iv, jv, BRid(iv), BRid(jv), ret, SINR, randomval, BLER(2,ret), distance(iv,jv));
% %             fprintf(msg);
% %         end
%         noErr = 0;
%     end
% end

end