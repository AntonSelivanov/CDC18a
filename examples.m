% This MATLAB program checks the feasibility of LMIs from Theorems 1 and 2 of the paper 
% A. Selivanov and E. Fridman, "Robust sampled-data implementation of PID controller," in 57th Conference on Decision and Control, 2018. 
%% Example 1 
a1=8.4; a2=0; b=35.71;              % parameters of (1)
kpbar=-10; kibar=-40; kdbar=-.65;   % parameters of (2) 
alpha=5;                            % decay rate 

% Periodic sampling 
h=.019;     % sampling period 
disp([char(10) 'Example 1'])
if LMI_CDC18a_th2(a1,a2,b,kpbar,kibar,kdbar,alpha,h,0)
    disp(['Theorem 1: feasible for h=' num2str(h)])
    disp(['kp=' num2str(kpbar+kdbar/h) ', ki=' num2str(kibar) ', kd=' num2str(-kdbar/h)])
else
    disp(['Theorem 1: not feasible for h=' num2str(h)])
end

% Event-based sampling 
h=.016;     % sampling period 
sigma=.02;	% event-triggering threshold 
if LMI_CDC18a_th2(a1,a2,b,kpbar,kibar,kdbar,alpha,h,sigma)
    disp(['Theorem 2: feasible for h=' num2str(h) ', sigma=' num2str(sigma)])
    disp(['kp=' num2str(kpbar+kdbar/h) ', ki=' num2str(kibar) ', kd=' num2str(-kdbar/h)])
else
    disp(['Theorem 2: not feasible for h=' num2str(h) ', sigma=' num2str(sigma)])
end
%% Example 2 
a1=[.01248 9.251]; a2=[5.862 22.19]; b=[.03707 0.04612];    % parameters of (1)
kpbar=-516.6; kibar=-143.8; kdbar=-765.5;                   % parameters of (2) 
alpha=.1;                                                   % decay rate 

% Periodic sampling 
h=.023;     % sampling period 
disp([char(10) 'Example 2'])
if LMI_CDC18a_th2(a1,a2,b,kpbar,kibar,kdbar,alpha,h,0)
    disp(['Theorem 1: feasible for h=' num2str(h)])
    disp(['kp=' num2str(kpbar+kdbar/h) ', ki=' num2str(kibar) ', kd=' num2str(-kdbar/h)])
else
    disp(['Theorem 1: not feasible for h=' num2str(h)])
end

% Event-based sampling 
h=.016;     % sampling period 
sigma=.1;	% event-triggering threshold 
if LMI_CDC18a_th2(a1,a2,b,kpbar,kibar,kdbar,alpha,h,sigma)
    disp(['Theorem 2: feasible for h=' num2str(h) ', sigma=' num2str(sigma)])
    disp(['kp=' num2str(kpbar+kdbar/h) ', ki=' num2str(kibar) ', kd=' num2str(-kdbar/h)])
else
    disp(['Theorem 2: not feasible for h=' num2str(h) ', sigma=' num2str(sigma)])
end
