function [x] = ForwardSim(x_init,t_array,u_array,sys,plot_result)
%create zero array to store state values
if nargin>5
    error('Too many inputs')
end
if nargin<4
    error('Not enough inputs')
end

switch nargin
    case 4
        plot_result=false;
end

x=zeros(length(x_init),length(t_array));
x_current=x_init;
x(:,1)=x_current;
u_current=u_array(:,1);

for i=2:length(t_array)
    x_dot=ComputeStateDerivatives(x_current,u_current,sys);
    x_current=x_current+x_dot.*(t_array(i)-t_array(i-1));
    x(:,i)=x_current;
    u_current=u_array(:,i);    
end

[x_loc,y_loc]=theta2xy(x(1,:),x(3,:),sys);

if plot_result
    subplot(3,2,1)
    plot(t_array,x(1,:),'k')
    ylabel('theta1')
    subplot(3,2,2)
    plot(t_array,x(3,:),'k')
    ylabel('theta2')   
    subplot(3,2,3)
    plot(t_array,x(2,:),'b')
    ylabel('omega1')
    subplot(3,2,4)
    plot(t_array,x(4,:),'b')
    ylabel('omega2')
    subplot(3,2,5)
    plot(t_array,x_loc,'r')
    ylabel('x')
    subplot(3,2,6)
    plot(t_array,y_loc,'r')
    ylabel('y')
end
end

