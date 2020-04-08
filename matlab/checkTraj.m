function[val,cost]= checkTraj(traj,origin,resolution,costmap)
    val=true;
    cost=0;
    [idx1, idx2]=XYtoMat(origin,traj.states(1,:),traj.states(2,:),resolution);
    if(sum(idx1<=0)>0 || sum(idx2 <=0)>0 || sum(idx1>size(costmap,1))>0 || sum(idx2>size(costmap,2))>0)
        val=false;
        return;
    end
    cell_cost=diag(double(costmap(idx1,idx2)));
    if(sum(cell_cost >= 240)>0)
        val=false;
        return;
    end
    cost=sum(cell_cost);
    
    x = [traj.states(1,:)];
    y = [traj.states(2,:)];
  
    
end