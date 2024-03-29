clear;
close all;

%%sss
% density = [84, 168, 336, 672, 1332];
% density = [84,150, 168, 300, 336,450, 600, 672, 750];
density = [150,300,450,600,750];

% [10,23]
% meanCBP_P = [0.368655, 0.54328, 0.678549, 0.832191];
% meanCBP_P = [0.368655,	0.523782,	0.54328,	0.67968,	0.678549,	0.749535,	0.793858,	0.832191,	0.87942];
%[5, 23]
% meanCBP_P = [0.36896, 0.537938, 0.648577, 0.720345, 0.9477]; %84
% meanCBP_P = [0.515002,	0.646759,	0.677244,	0.694482,	0.786422]; %150, th94
meanCBP_P = [0.5193	0.605871	0.624849	0.670357	0.756275]; %150, th107

% meanCBP_P = [0.36896,	0.522449,	0.537938,	0.651176,	0.648577,	0.674837,	0.677572,	0.720345,	0.784849];
%[0.1, 0.3]
% meanCBP_C = [0.362297, 0.491971, 0.568216, 0.717245, 0.86866];
%[0.1, 0.2]
% meanCBP_I = [0.368655, 0.493913, 0.626265, 0.739476, 0.940276]; %84
% meanCBP_I = [0.484822,	0.641377,	0.678376,	0.715806,	0.775135]; %150 th 94
meanCBP_I = [0.497645	0.60334	0.662684	0.707555	0.75857]; %150 th 107


%5:5
% power에서 만든 cbp -rate가 볼거임!
% meanCBP_C = [0.449499,	0.548094,	0.602894,0.654316,	0.666333];
% rate에서 만든cbp
% meanCBP_C = [0.441519,	0.603621,	0.725131,	0.825949,	0.898249];

%수정후
% meanCBP_C = [0.491563,	0.624738,	0.702574,	0.771768	,0.806535];
% %수정전
% meanCBP_C = [0.491414,	0.628792,	0.699156,	0.775085,	0.808256];

%3:7
% 수정전
% meanCBP_C = [0.489206,	0.627433,	0.686464,	0.766331,	0.833979];
% 수정후
% meanCBP_C = [0.446322	0.577465	0.65672	0.742057	0.779975];

%7:3
% meanCBP_C = [0.502915,	0.632159,	0.722351,	0.74915,	0.766682];

%% numOfUEs 기준으로

hold on;
grid on;
% TxPower
x = density;
y1 = meanCBP_P;
axis([100 800 0.1 1.0]);
plot(x,y1, 'o-');

% hold on;
y2 = meanCBP_I;
axis([100 800 0.1 1.0]);

plot(x,y2, 'ro-');
% 
% hold on;
% y3 = meanCBP_C;
% plot(x,y3, 'ko-');

title('\bf\fontsize{14}Fig 2. Mean CBR for number of UEs'); 

% legend('Power3Rate7', 'Power5Rate5', 'Power7Rate3', 'Location', 'NorthWest');

% legend('Power5', 'ITT0.2', 'combine', 'Location', 'NorthWest');
% legend('totalCBR', 'PowerCBR','RateCBR', 'Location', 'NorthWest');
legend('Power[5,23]', 'Location', 'NorthWest');


xlabel('\bfNumber of UEs');   
ylabel('\bfMean CBR');      % CBP


