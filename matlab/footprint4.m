function [footprint_mat]=footprint4(traj,sensor,dim,simPar,sample_frac,footprint,offset)
%% Function evaluates the footprint of the robot
% traj:  trajectory structure(evaluates for array or single trajectory)
% sensor: sensor information
% dim : dimension of the map
% simPar.origin: simPar.origin of the map
% simPar.resolution: map simPar.resolution
%%
if(nargin==4)
    sample_frac=1;
end

if( nargin == 6)
    offset = 0;
end

    footprint_mat=zeros(dim(1),dim(2),length(traj));
    j_c=sensor.range/simPar.resolution+1;
    k_c=sensor.range/simPar.resolution+1;
    
    state=[traj.states];
    %state=reshape(state,size(traj(1).states,1),size(traj(1).states,2),[]);
    traj_num=[traj.num];
    
    traj_num = [0,cumsum(traj_num)];
    
    [m,n]=XYtoMat(simPar.origin,state(1,:,:),state(2,:,:),simPar.resolution);
    m=squeeze(m);
    n=squeeze(n);
    
    rot=squeeze(mod(round(state(3,:,:)*180/pi)+180-offset,360)+1);
    for(k=1:length(traj))    
        mat=zeros(dim);
        
        boundl_x=m-sensor.range/simPar.resolution;
        boundh_x=m+sensor.range/simPar.resolution;
        
        boundl_y=n-sensor.range/simPar.resolution;
        boundh_y=n+sensor.range/simPar.resolution;
        
        for (j=traj_num(k)+1:sample_frac:traj_num(k+1))
            if(boundl_x(j)<=0)
                if(boundl_y(j)<=0)
                    footprint_mat(1:boundh_x(j),1:boundh_y(j),k)=footprint_mat(1:boundh_x(j),1:boundh_y(j),k)+footprint(-boundl_x(j)+2:size(footprint,1),-boundl_y(j)+2:size(footprint,2),rot(j));
                elseif(boundh_y(j)>dim(2))
                    footprint_mat(1:boundh_x(j),boundl_y(j):dim(2),k)=footprint_mat(1:boundh_x(j),boundl_y(j):dim(2),k)+footprint(-boundl_x(j)+2:size(footprint,1),1:size(footprint,2)-(boundh_y(j)-dim(2)),rot(j));
                else
                    footprint_mat(1:boundh_x(j),boundl_y(j):boundh_y(j),k)=footprint_mat(1:boundh_x(j),boundl_y(j):boundh_y(j),k)+footprint(-boundl_x(j)+2:size(footprint,1),:,rot(j));
                end
            elseif(boundh_x(j)>dim(1))
                if(boundl_y(j)<=0)
                    footprint_mat(boundl_x(j):dim(1),1:boundh_y(j),k)=footprint_mat(boundl_x(j):dim(1),1:boundh_y(j),k)+footprint(1:size(footprint,1)-(boundh_x(j)-dim(1)),-boundl_y(j)+2:size(footprint,2),rot(j));
                elseif(boundh_y(j)>dim(2))
                    footprint_mat(boundl_x(j):dim(1),boundl_y(j):dim(2),k)= footprint_mat(boundl_x(j):dim(1),boundl_y(j):dim(2),k)+footprint(1:size(footprint,1)-(boundh_x(j)-dim(1)),1:size(footprint,2)-(boundh_y(j)-dim(2)),rot(j));
                else
                    footprint_mat(boundl_x(j):dim(1),boundl_y(j):boundh_y(j),k)= footprint_mat(boundl_x(j):dim(1),boundl_y(j):boundh_y(j),k)+footprint(1:size(footprint,1)-(boundh_x(j)-dim(1)),:,rot(j));
                end
            elseif(boundh_y(j)>dim(2))
                footprint_mat(boundl_x(j):boundh_x(j),boundl_y(j):dim(2),k)=footprint_mat(boundl_x(j):boundh_x(j),boundl_y(j):dim(2),k)+footprint(:,1:size(footprint,2)-(boundh_y(j)-dim(2)),rot(j));
            elseif(boundl_y(j)<=0)
                footprint_mat(boundl_x(j):boundh_x(j),1:boundh_y(j),k)= footprint_mat(boundl_x(j):boundh_x(j),1:boundh_y(j),k)+footprint(:,-boundl_y(j)+2:size(footprint,2),rot(j));
            else
                footprint_mat(boundl_x(j):boundh_x(j),boundl_y(j):boundh_y(j),k)=footprint_mat(boundl_x(j):boundh_x(j),boundl_y(j):boundh_y(j),k)+footprint(:,:,rot(j));
            end
        end
    end
    footprint_mat=reshape(footprint_mat,dim(1)*dim(2),[]);
  
end

