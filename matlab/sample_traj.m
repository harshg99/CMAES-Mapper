function [trajs,control]=sample_traj(mean,variance,simPar,costMap)

% Samples a trajectory parametrised on control( control tuples
% mean: (n,1) mean vector of control
% variance: (n,n) variance of control
% simPar: Simualtion Parameters (start,tstep,tim)
% origin: Origin of thee robot
% resolution
%costmap for checking trajectory
if(nargin<4)
    control=mvnrnd(mean,variance,num);
    return;
end
i=1;
k=1;


while(i<=simPar.numTraj)
    samp_control=mvnrnd(mean,variance,1);
    k=k+1;
    traj=toTraj(simPar.start,samp_control,simPar.t_step,simPar.time);
    [val,cost]=checkTraj(traj,simPar.origin,simPar.resolution,costMap);
    %%Collision with obstacles
    if(val)
        traj.cost=cost;
        trajs(i)=traj;
        control(i,:)=samp_control;
        i=i+1;      
    end
end
end