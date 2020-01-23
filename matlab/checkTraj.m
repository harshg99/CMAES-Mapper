function[val,cost]= checkTraj(traj,origin,resolution,costmap)
    val=true;
    cost=0;
    for(j=1:size(traj.states,2))
        [idx1, idx2]=XYtoMat(origin,traj.states(1,j),traj.states(2,j),resolution);
        if(idx1<=0 || idx2 <=0 ||idx1>size(costmap,1)||idx2>size(costmap,1))
            val=false;
            return;
        end
        cell_cost=double(costmap(idx1,idx2));
         if(cell_cost >= 253)
            val=false;
            return;
        end
        cost=cost+cell_cost;    
    end
end