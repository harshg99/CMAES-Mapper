function [footprint_mat]=footprint3(traj,sensor,dim,simPar,sample_frac)
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

if( length(traj) == 1)
    footprint_mat=zeros(dim);
    j_c=sensor.range/simPar.resolution+1;
    k_c=sensor.range/simPar.resolution+1;
    footprint_c=reshape(0:(2*j_c-1)*(2*k_c-1)-1,[int32(2*j_c-1) int32(2*k_c-1)]);
    for(j=1:5:size(traj.states,2))
        mat=zeros(dim);
        state=traj.states(:,j);
        
        [m,n]=XYtoMat(simPar.origin,state(1),state(2),simPar.resolution);
        
        %footprint=zeros(2*sensor.range/simPar.resolution+1);
        
        
        footprint = (((footprint_c/(2*j_c-1)+1-j_c).^2+(mod(footprint_c,2*k_c-1)+1-k_c).^2)<=power(int32(sensor.range/simPar.resolution),2));
        % angle=abs(atan2d(j_c-(mod(footprint_c,2*j_c-1)+1),(footprint_c/(2*k_c-1)+1)-k_c));
        footprint= footprint & (abs((atan2d(j_c-(mod(footprint_c,2*j_c-1)+1),(footprint_c/(2*k_c-1)+1)-k_c)-state(3)*180/pi))<=sensor.fov);
        
        boundl_x=m-int32(sensor.range/simPar.resolution);
        boundh_x=m+int32(sensor.range/simPar.resolution);
        
        boundl_y=n-int32(sensor.range/simPar.resolution);
        boundh_y=n+int32(sensor.range/simPar.resolution);
        
        if(boundl_x<=0)
            if(boundl_y<=0)
                footprint_mat(1:boundh_x,1:boundh_y)=footprint_mat(1:boundh_x,1:boundh_y)+footprint(-boundl_x+2:size(footprint,1),-boundl_y+2:size(footprint,2));
            elseif(boundh_y>dim(2))
                footprint_mat(1:boundh_x,boundl_y:dim(2))=footprint_mat(1:boundh_x,boundl_y:dim(2))+footprint(-boundl_x+2:size(footprint,1),1:size(footprint,2)-(boundh_y-dim(2)));
            else
                footprint_mat(1:boundh_x,boundl_y:boundh_y)=footprint_mat(1:boundh_x,boundl_y:boundh_y)+footprint(-boundl_x+2:size(footprint,1),:);
            end
        elseif(boundh_x>dim(1))
            if(boundl_y<=0)
                footprint_mat(boundl_x:dim(1),1:boundh_y)=footprint_mat(boundl_x:dim(1),1:boundh_y)+footprint(1:size(footprint,1)-(boundh_x-dim(1)),-boundl_y+2:size(footprint,2));
            elseif(boundh_y>dim(2))
                footprint_mat(boundl_x:dim(1),boundl_y:dim(2))= footprint_mat(boundl_x:dim(1),boundl_y:dim(2))+footprint(1:size(footprint,1)-(boundh_x-dim(1)),1:size(footprint,2)-(boundh_y-dim(2)));
            else
                footprint_mat(boundl_x:dim(1),boundl_y:boundh_y)= footprint_mat(boundl_x:dim(1),boundl_y:boundh_y)+footprint(1:size(footprint,1)-(boundh_x-dim(1)),:);
            end
        elseif(boundh_y>dim(2))
            footprint_mat(boundl_x:boundh_x,boundl_y:dim(2))=footprint_mat(boundl_x:boundh_x,boundl_y:dim(2))+footprint(:,1:size(footprint,2)-(boundh_y-dim(2)));
        elseif(boundl_y<=0)
            footprint_mat(boundl_x:boundh_x,1:boundh_y)= footprint_mat(boundl_x:boundh_x,1:boundh_y)+footprint(:,-boundl_y+2:size(footprint,2));
        else
            footprint_mat(boundl_x:boundh_x,boundl_y:boundh_y)=footprint_mat(boundl_x:boundh_x,boundl_y:boundh_y)+footprint;
        end
        %footprint_mat=footprint_mat+mat;
        
    end
    
else
    footprint_mat=zeros(dim(1),dim(2),length(traj));
    j_c=sensor.range/simPar.resolution+1;
    k_c=sensor.range/simPar.resolution+1;
    angles=[0:1:359]-180;
    footprint_c=repmat([0:(2*j_c-1)*(2*k_c-1)-1]',1,length(angles));
    
    state=[traj.states];
    %state=reshape(state,size(traj(1).states,1),size(traj(1).states,2),[]);
    traj_num=[traj.num];
    
    traj_num = [0,cumsum(traj_num)];
    
    [m,n]=XYtoMat(simPar.origin,state(1,:,:),state(2,:,:),simPar.resolution);
    m=squeeze(m);
    n=squeeze(n);
    
 
    footprint = (((footprint_c/(2*j_c-1)+1-j_c).^2+(mod(footprint_c,2*k_c-1)+1-k_c).^2)<=power(int32(sensor.range/simPar.resolution),2));
        % angle=abs(atan2d(j_c-(mod(footprint_c,2*j_c-1)+1),(footprint_c/(2*k_c-1)+1)-k_c));
    
    %toc;
    footprint= footprint & (abs((atan2d(j_c-(mod(footprint_c,2*j_c-1)+1),(footprint_c/(2*k_c-1)+1)-k_c)-angles))<=sensor.fov);
    footprint=reshape(footprint,2*j_c-1,2*k_c-1,[]);
    %toc;
    
    rot=squeeze(mod(round(state(3,:,:)*180/pi),360)+1);
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

