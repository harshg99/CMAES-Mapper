function [trajs,control]=sample_traj_PRM(map,GMDist,numNodes,numTraj,origin)

% Samples a trajectory parametrised on control( control tuples
% mean: (n,1) mean vector of control
% variance: (n,n) variance of control
% origin: Origin for the map
% GMDist: 
%costmap for checking trajectory
if(nargin<4)
    control=mvnrnd(mean,variance,num);
    return;
end
i=1;
k=1;
0
a = binaryOccupancyMap(map,1/resolution);
rm = PRM(a,numNodes);
rm.GMDist = GMDist;
show(rm);
toc;

while (i<=numTraj)
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