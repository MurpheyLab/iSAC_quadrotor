function [ xdesired ] = desired_trajectory( t )
%specify desired trajectory
global param
 
  x =6*sin(t/3);
y = -6*sin(t/3).*cos(t/3);
z = 6*cos(t/3);




xdesired = [x;y;z;0;0;0;0;0;0;0;0;0];
end

