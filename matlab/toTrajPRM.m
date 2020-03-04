function [traj]=toTrajPRM(trajectory,simPar)

% Generates a trajectory based on control
% Control arranged as (u_t1,u_theta1,u_t2,u_theta2,...)
% start configuration (x,y,theta)

%% Initialising trajectory
traj_.states=cell(1,length(trajectory)-1);
traj_.num=1;
traj_.cost=0;

for j=1:length(trajectory)-1
    angle = atan2d(trajectory(2,j+1)-trajectory(2,j).trajectory(1,j+1)-trajectory(1,j));
    states = [ trajectory(1,j):simPar.step*cos(angle):trajectory(1,j+1);
               trajectory(2,j):simPar.step*sin(angle):trajectory(2,j+1);
               angle];
    traj_.states{1}=states;
end

traj_.states=cell2mat(traj_.states);
traj_.num=length(traj_.states);
traj(1)=traj_;
end
