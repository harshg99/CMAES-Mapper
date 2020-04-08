function [traj_]=toTrajPRM(trajectory,simPar,costmap)

% Generates a trajectory based on control
% Control arranged as (u_t1,u_theta1,u_t2,u_theta2,...)
% start configuration (x,y,theta)

%% Initialising trajectory
traj_.states=cell(1,length(trajectory)-1);
traj_.num=1;
traj_.cost=0;
traj_.length = 0;
previous_angle = 0;
for j=1:length(trajectory)-1
    angle = atan2(trajectory(2,j+1)-trajectory(2,j),trajectory(1,j+1)-trajectory(1,j));
    states = [ trajectory(1,j):simPar.step*cos(angle):trajectory(1,j+1);
               trajectory(2,j):simPar.step*sin(angle):trajectory(2,j+1);];
    traj_.length = traj_.length + norm([trajectory(:,j+1) - trajectory(:,j);(angle - previous_angle)]);
    states(3,:)=angle;
    previous_angle = angle;
    traj_.states{j}=states;
end

traj_.states=cell2mat(traj_.states);
traj_.num=length(traj_.states);

[idx1, idx2]=XYtoMat(simPar.origin,traj_.states(1,:),traj_.states(2,:),simPar.resolution);
cell_cost=diag(double(costmap(idx1,idx2)));
traj_.cost=sum(cell_cost);

% default frame is different

end
