function flag=LMI_CDC18a_th2(a1,a2,b,kpbar,kibar,kdbar,alpha,h,sigma)
% This MATLAB program checks the feasibility of LMIs from Proposition 1 of the paper 
% A. Selivanov and E. Fridman, "Robust sampled-data implementation of PID controller," in 57th Conference on Decision and Control, 2018. 

% The program uses YALMIP parser (http://users.isy.liu.se/johanl/yalmip/)

% Input: 
% a1,a2,b           - parameters of (1)
% kpbar,kibar,kdbar - observer gain
% alpha             - decay rate
% h                 - sampling period
% sigma             - event-triggering threshold  

% Output: 
% flag =1 if feasible, =0 otherwise
%% Decision variables and notations 
P=sdpvar(3); 
S=sdpvar(3); 
sdpvar W R omega 
G=h^2*exp(2*alpha*h)*S+h^2*[0 0 0; 0 1 0; 0 0 0]*(1/4*R+exp(2*alpha*h)*W); 
%% LMIs 
LMIs=P>=0; 
for i=1:length(a1) 
    for j=1:length(a2) 
        for k=1:length(b) 
            A=[0 1 0; -a2(j)+b(k)*kpbar -a1(i)+b(k)*kdbar b(k)*kibar; 1 0 0]; 
            Av=[0 0 0; b(k)*kpbar 0 b(k)*kibar; 1 0 0]; 
            B=[0; b(k)*kdbar; 0]; 
            
            Phi=blkvar; 
            Phi(1,1)=P*A+A'*P+2*alpha*P; 
            Phi(1,2)=P*Av; 
            Phi(1,3)=P*B; 
            Phi(1,4)=P*B; 
            Phi(1,5)=A'*G; 
            Phi(1,6)=P*B/kdbar; 
            Phi(1,7)=sigma*omega*[kpbar; kdbar; kibar]; 
            Phi(2,2)=-pi^2/4*S; 
            Phi(2,5)=Av'*G; 
            Phi(2,7)=sigma*omega*[kpbar; 0; kibar]; 
            Phi(3,3)=-W*pi^2/4*exp(-2*alpha*h); 
            Phi(3,5)=B'*G; 
            Phi(3,7)=sigma*omega*kdbar; 
            Phi(4,4)=-R*exp(-2*alpha*h); 
            Phi(4,5)=B'*G; 
            Phi(4,7)=sigma*omega*kdbar; 
            Phi(5,5)=-G; 
            Phi(5,6)=G*B/kdbar; 
            Phi(6,6)=-omega; 
            Phi(7,7)=-sigma*omega; 
            Phi=sdpvar(Phi); 
            
            LMIs=[LMIs, Phi<=0];  %#ok<AGROW>
        end
    end
end
%% Solution of LMIs
options=sdpsettings('solver','lmilab','verbose',0); 
sol=optimize(LMIs,[],options); 

flag=0; 
if sol.problem==0
    [primal,~]=check(LMIs); 
    flag=min(primal)>=0; 
else
    yalmiperror(sol.problem) 
end