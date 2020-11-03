function [x_dot] = ComputeStateDerivatives(x,u,sys)
%----------------------------------------------------------------------------
% input of the function
% x is the states of the system in the form [theta1;theta1_dot;theta2;theta2_dot]
% u=[u1;u2] that within acturator constraints
% sys is the structure of system parameters
%---------------------------------------------------------------------------
% output of the function
% x_dot output of the dynamical system in the form:
% [theta1_dot; theta1_dotdot; theta2_dot;theta2_dotdot]

%initial check
for i=1:length(u)
%     if u(i)<sys.actuator{i}.act_lim(1)-0.01 || u(i)>sys.actuator{i}.act_lim(2)+0.01
%         keyboard
%         error('actuator %i exceed limit',i)
%     end
end

%extract parameters
m1=sys.para.m1;
m2=sys.para.m2;
L1=sys.para.L1;
L2=sys.para.L2;
I1=sys.para.I1;
I2=sys.para.I2;
rc1=sys.para.rc1;
rc2=sys.para.rc2;
g=sys.para.g;

%System Mass Matrix
% [0 A1 0 A2
%  1 0  0 0
%  0 A3 0 A4
%  0 0  1 0  ]

A1=I1+m1*rc1^2+m2*L1^2+m2*L1*rc2*cos(x(1)-x(3));
A2=I2+m2*rc2^2+m2*L1*rc2*cos(x(1)-x(3));
A3=m2*L1*rc2*cos(x(1)-x(3));
A4=I2+m2*rc2^2;

M=[0 A1 0 A2; 1 0 0 0;...
   0 A3 0 A4; 0 0 1 0];

f1=m2*L1*rc2*sin(x(1)-x(3))*x(2)^2-m2*L1*rc2*sin(x(1)-x(3))*x(4)^2-(m1*rc1+m2*L1)*g*cos(x(1))-m2*rc2*g*cos(x(3));
f2=x(2);
f3=m2*L1*rc2*sin(x(1)-x(3))*x(2)^2-m2*rc2*g*cos(x(3));
f4=x(4);
F=[f1;f2;f3;f4];
U=[u(1);0;u(2);0];
x_dot=M\(F+U);


end

