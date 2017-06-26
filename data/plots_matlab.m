clear all

global param 
global master

%Read and format states and time
states = csvread('states.csv');
time = states(:,1);
states(:,1) = [];
%Read and format actions
master.all_actions = csvread('actions.csv');
master.all_actions(:,end) = [];

% Load all settings in struct "param" 
param = settings();


%Plot desired trajectory
t = 0:0.01:param.tf;
for i=1:length(t)
    des_traj(i,:) = desired_trajectory( t(i) );
end



figure();
hold on
plot(time,states(:,1:6))
plot(t, des_traj(:,1:6), '--', 'linewidth', 2)
 
xlabel('time (s)')
title('States')
legend('x','y','z','phi','theta','psi','xref','yref','zref','phiref','thetaref','psiref');


plot_control();