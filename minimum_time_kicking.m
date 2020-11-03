clear
clc
close all

%Define system parameter
sys.para.m1=7; sys.para.L1=0.4;sys.para.I1=0.12;sys.para.rc1=0.2294;

sys.para.m2=3.51; sys.para.L2=0.43;sys.para.I2=0.05;sys.para.rc2=0.2438;
%  sys.para.m2=10; sys.para.L2=0.003 ;sys.para.I2=0.01;sys.para.rc2=0.001;
sys.para.g=9.8;
sys.para.nstates=4;
sys.para.ncontrol=2;
actuator1.name="hip torque";
actuator1.act_lim=[-300 200];
actuator2.name="knee torque";
actuator2.act_lim=[-100 300];

sys.actuator={actuator1 actuator2};

clear actuator1 actuator2
%%
%Define initial pose (in radian)
theta1_init=270/180*pi;
theta1_dot_init=0;
theta2_init=265/180*pi;
theta2_dot_init=0;

x_init=[theta1_init;theta1_dot_init;theta2_init;theta2_dot_init];
%% Formulate optimal control problem
nodes=30;
target=[0.8;-0.21];
% generating initial guess
tf_ig=1;
t_ig_array=linspace(0,tf_ig,nodes);
u_ig_array=ones(2,nodes);
% u_ig_array(1,1:2)=-sys.actuator{1}.act_lim(2)/100;
% u_ig_array(2,1:2)=-sys.actuator{2}.act_lim(2)/100;
% x=ForwardSim(x_init,t_ig_array,u_ig_array,sys);

% initial guess structure
% [state 1 state 2 state 3 state 4 u1 u2 tf]
% initial_guess=reshape(x',1,sys.para.nstates*nodes);
initial_guess=[reshape(u_ig_array',1,sys.para.ncontrol*nodes)];
initial_guess=[initial_guess tf_ig];

% generating design variable constraints lb ub
% ub=[2*pi*ones(1,nodes) 100*ones(1,nodes) 2*pi*ones(1,nodes) 100*ones(1,nodes) sys.actuator{1}.act_lim(2)*ones(1,nodes) sys.actuator{2}.act_lim(2)*ones(1,nodes) 1];
% lb=[pi/2*ones(1,nodes) -100*ones(1,nodes) pi/2*ones(1,nodes) -100*ones(1,nodes) sys.actuator{1}.act_lim(1)*ones(1,nodes) sys.actuator{2}.act_lim(1)*ones(1,nodes) 0];
ub=[sys.actuator{1}.act_lim(2)*ones(1,nodes) sys.actuator{2}.act_lim(2)*ones(1,nodes) 2];
lb=[sys.actuator{1}.act_lim(1)*ones(1,nodes) sys.actuator{2}.act_lim(1)*ones(1,nodes) 0];

% generating nonlinear constraint function 
%(initial states,dyanmical system equation, torque limit and
% final x,y location)

% defining objective function
% to minimize the last element of design variable, final time
objfun=@(guess) guess(end);
% setting up optimizer
confun=@(guess) cons(guess,x_init,nodes,sys,target);
% solving the NLP
 % start a timer
tic;
options = optimset('algorithm','interior-point','TolFun',1e-6,'TolX',1e-6, ...
                   'TolCon',1e-8,'FinDiffType','central','MaxFunEvals',1e8,'MaxIter',1e5, ...
                   'Hessian','bfgs','display','iter');
[Xopt,fval,exitflag,output] = fmincon(objfun,initial_guess,[],[],[],[],lb,ub,confun,options);
% options=optimset('UseParallel',true);
% [Xopt,fval,exitflag,output,population,scores] = ga(objfun,length(initial_guess),[],[],[],[],lb,ub,confun,options);
% stop the timer
runtime = toc;

%%
nstates=sys.para.nstates;
ncontrol=sys.para.ncontrol;
tf=Xopt(end);
time_array=linspace(0,tf,nodes);
Xopt=Xopt(1:end-1);

u=reshape(Xopt,nodes,ncontrol)';
x=ForwardSim(x_init,time_array,u,sys);
% x=Xopt(1:nstates,:);

[x_loc,y_loc]=theta2xy(x(1,:),x(3,:),sys);
    subplot(4,2,1)
    plot(time_array,x(1,:),'k')
    ylabel('theta1')
    subplot(4,2,2)
    plot(time_array,x(3,:),'k')
    ylabel('theta2')   
    subplot(4,2,3)
    plot(time_array,x(2,:),'b')
    ylabel('omega1')
    subplot(4,2,4)
    plot(time_array,x(4,:),'b')
    ylabel('omega2')
    subplot(4,2,5)
    plot(time_array,x_loc,'r')
    ylabel('x')
    subplot(4,2,6)
    plot(time_array,y_loc,'r')
    ylabel('y')
    subplot(4,2,7)
    plot(time_array,u(1,:),'g')
    ylabel('u1')
    subplot(4,2,8)
    plot(time_array,u(2,:),'g')
    ylabel('u2')
    
    