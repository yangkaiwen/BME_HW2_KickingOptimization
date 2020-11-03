function [c,ceq] = cons(guess,init,nodes,sys,target)
%target=[x;y]
% nstates=sys.para.nstates;
ncontrol=sys.para.ncontrol;
tf=guess(end);
time_array=linspace(0,tf,nodes);
guess=guess(1:end-1);

u=reshape(guess,nodes,ncontrol)';
x=ForwardSim(init,time_array,u,sys);
theta1=x(1,:);
theta2=x(3,:);

% ceq=[(x(:,1)-init)']; % prescripted initial states
theta1_end=x(1,nodes);
theta2_end=x(3,nodes);
[xloc,yloc]=theta2xy(theta1_end,theta2_end,sys); % evaluate final ankle location
ceq=[xloc-target(1,1) yloc-target(2,1)]; % final end-effector constraint

%drive x(:,i) with u(i) , the result has to equal x(:,i+1)

% for i=1:nodes-1
%     x_dot_1=ComputeStateDerivatives(x(:,i),u(:,i),sys);
%     x_dot_2=ComputeStateDerivatives(x(:,i+1),u(:,i+1),sys);
%     x_dot=(x_dot_1+x_dot_2)/2;
%     x_step=x_dot+x_dot.*(time_array(i+1)-time_array(i));
%     diff=x_step-x(:,i+1);
%     ceq=[ceq diff'];
% end

c=[theta1-3*pi theta2-3*pi -theta2 -theta1];
% c=u(1,:)-sys.actuator{1}.act_lim(2);
% c=[c sys.actuator{1}.act_lim(1)-u(1,:)];
% c=[c u(2,:)-sys.actuator{2}.act_lim(2)];
% c=[c sys.actuator{2}.act_lim(1)-u(2,:)];
% c=[c -tf];




end

