function [trajs,control]=sample_traj(mean,variance,simPar,costMap,velocity)

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
    samp=mvnrnd(mean,variance,1);
    k=k+1;
    samp_control = zeros(1,size(samp,2));
    
    if( nargin == 5)
        samp_control(1,2:2:2*size(samp,2)) = samp;
        samp_control(1,1:2:2*size(samp,2)-1) = velocity;
    else
        samp_control = samp;
    end
        
    traj=toTraj(simPar.start,samp_control,simPar.t_step,simPar.time);
    [val,cost]=checkTraj(traj,simPar.origin,simPar.resolution,costMap);
    %%Collision with obstacles
    if(val)
        traj.cost=cost;
        trajs(i)=traj;
        control(i,:)=samp;
        i=i+1;      
    end
end
end