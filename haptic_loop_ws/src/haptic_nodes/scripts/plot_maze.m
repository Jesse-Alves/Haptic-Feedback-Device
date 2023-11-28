%% Plot the Maze generated.
clear all; close all; clc;

%M = csvread("sphere_centers.csv",',')
M = readmatrix("maze1_HighCost.csv");

figure
for ii=1:length(M)
    plot(M(ii,2),M(ii,3),'*','LineWidth',30)
    hold on
end
grid on
set(gca, 'YDir','reverse')
set(gca, 'XDir','reverse')

title('Center of Spheres','FontSize',20)
xlabel('x','FontSize',30)
ylabel('y','FontSize',30)

return
%% Generate the trajectory inside of Maze
clc

traj1 = [linspace(0.0198,-0.0301,250)' ...
         0.1043*ones(250,1)];

traj2 = [-0.0301*ones(250,1) ...
         linspace(0.1043,0.1331,250)'];

traj1_inv = flip(traj1);
traj2_inv = flip(traj2);

traj = [traj1;traj2;traj2_inv;traj1_inv]

writematrix(traj,"traj_maze1_HighCost.csv")




