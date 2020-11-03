function [x,y] = theta2xy(theta1,theta2,sys)
 x=cos(theta1)*sys.para.L1+cos(theta2)*sys.para.L2;
 y=sin(theta1)*sys.para.L1+sin(theta2)*sys.para.L2;
end

