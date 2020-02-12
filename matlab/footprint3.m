function [footprint_mat]=footprint3(traj,sensor,dim,origin,resolution,sample_frac)
%% Function evaluates the footprint of the robot
% traj:  trajectory structure(evaluates for array or single trajectory)
% sensor: sensor information
% dim : dimension of the map
% origin: origin of the map
% resolution: map resolution
%%
if(nargin==5)
    sample_frac=5;
end

if(length(traj)==1)
    footprint_mat=zeros(dim);
    j_c=sensor.range/resolution+1;
    k_c=sensor.range/resolution+1;
    footprint_c=reshape(0:(2*j_c-1)*(2*k_c-1)-1,[int32(2*j_c-1) int32(2*k_c-1)]);
    for(j=1:5:size(traj.states,2))
        mat=zeros(dim);
        state=traj.states(:,j);
        
        [m,n]=XYtoMat(origin,state(1),state(2),resolution);
        
        %footprint=zeros(2*sensor.range/resolution+1);
        
        
        footprint = (((footprint_c/(2*j_c-1)+1-j_c).^2+(mod(footprint_c,2*k_c-1)+1-k_c).^2)<=power(int32(sensor.range/resolution),2));
        % angle=abs(atan2d(j_c-(mod(footprint_c,2*j_c-1)+1),(footprint_c/(2*k_c-1)+1)-k_c));
        footprint= footprint & (abs((atan2d(j_c-(mod(footprint_c,2*j_c-1)+1),(footprint_c/(2*k_c-1)+1)-k_c)-state(3)*180/pi))<=sensor.fov);
        
        boundl_x=m-int32(sensor.range/resolution);
        boundh_x=m+int32(sensor.range/resolution);
        
        boundl_y=n-int32(sensor.range/resolution);
        boundh_y=n+int32(sensor.range/resolution);
        
        if(boundl_x<=0)
            if(boundl_y<=0)
                footprint_mat(1:boundh_x,1:boundh_y)=footprint_mat(1:boundh_x,1:boundh_y)+footprint(-boundl_x+2:size(footprint,1),-boundl_y+2:size(footprint,2));
            elseif(boundh_y>dim)
                footprint_mat(1:boundh_x,boundl_y:dim)=footprint_mat(1:boundh_x,boundl_y:dim)+footprint(-boundl_x+2:size(footprint,1),1:size(footprint,2)-(boundh_y-dim));
            else
                footprint_mat(1:boundh_x,boundl_y:boundh_y)=footprint_mat(1:boundh_x,boundl_y:boundh_y)+footprint(-boundl_x+2:size(footprint,1),:);
            end
        elseif(boundh_x>dim)
            if(boundl_y<=0)
                footprint_mat(boundl_x:dim,1:boundh_y)=footprint_mat(boundl_x:dim,1:boundh_y)+footprint(1:size(footprint,1)-(boundh_x-dim),-boundl_y+2:size(footprint,2));
            elseif(boundh_y>dim)
                footprint_mat(boundl_x:dim,boundl_y:dim)= footprint_mat(boundl_x:dim,boundl_y:dim)+footprint(1:size(footprint,1)-(boundh_x-dim),1:size(footprint,2)-(boundh_y-dim));
            else
                footprint_mat(boundl_x:dim,boundl_y:boundh_y)= footprint_mat(boundl_x:dim,boundl_y:boundh_y)+footprint(1:size(footprint,1)-(boundh_x-dim),:);
            end
        elseif(boundh_y>dim)
            footprint_mat(boundl_x:boundh_x,boundl_y:dim)=footprint_mat(boundl_x:boundh_x,boundl_y:dim)+footprint(:,1:size(footprint,2)-(boundh_y-dim));
        elseif(boundl_y<=0)
            footprint_mat(boundl_x:boundh_x,1:boundh_y)= footprint_mat(boundl_x:boundh_x,1:boundh_y)+footprint(:,-boundl_y+2:size(footprint,2));
        else
            footprint_mat(boundl_x:boundh_x,boundl_y:boundh_y)=footprint_mat(boundl_x:boundh_x,boundl_y:boundh_y)+footprint;
        end
        %footprint_mat=footprint_mat+mat;
        
    end
    
else
    footprint_mat=zeros(dim,dim,length(traj));
    j_c=sensor.range/resolution+1;
    k_c=sensor.range/resolution+1;
    angles=[0:1:359]-180;
    footprint_c=repmat([0:(2*j_c-1)*(2*k_c-1)-1]',1,length(angles));
    state=[traj.states];state=reshape(state,size(traj(1).states,1),size(traj(1).states,2),[]);
   % toc;
    [m,n]=XYtoMat(origin,state(1,1:sample_frac:size(traj(1).states,2),:),state(2,1:sample_frac:size(traj(1).states,2),:),resolution);
    m=squeeze(m);
    n=squeeze(n);
    
    %toc;
    footprint = (((footprint_c/(2*j_c-1)+1-j_c).^2+(mod(footprint_c,2*k_c-1)+1-k_c).^2)<=power(int32(sensor.range/resolution),2));
        % angle=abs(atan2d(j_c-(mod(footprint_c,2*j_c-1)+1),(footprint_c/(2*k_c-1)+1)-k_c));
    
    %toc;
    footprint= footprint & (abs((atan2d(j_c-(mod(footprint_c,2*j_c-1)+1),(footprint_c/(2*k_c-1)+1)-k_c)-angles))<=sensor.fov);
    footprint=reshape(footprint,2*j_c-1,2*k_c-1,[]);
    %toc;
    
    rot=squeeze(mod(round(state(3,:,:)*180/pi),360)+1);

    for(j=1:sample_frac:size(traj(1).states,2))        
        mat=zeros(dim);
        
        boundl_x=m((j-1)/sample_frac+1,:)-sensor.range/resolution;
        boundh_x=m((j-1)/sample_frac+1,:)+sensor.range/resolution;
        
        boundl_y=n((j-1)/sample_frac+1,:)-sensor.range/resolution;
        boundh_y=n((j-1)/sample_frac+1,:)+sensor.range/resolution;
        
        for(k=1:length(traj))
            if(boundl_x(k)<=0)
                if(boundl_y(k)<=0)
                    footprint_mat(1:boundh_x(k),1:boundh_y(k),k)=footprint_mat(1:boundh_x(k),1:boundh_y(k),k)+footprint(-boundl_x(k)+2:size(footprint,1),-boundl_y(k)+2:size(footprint,2),rot(j,k));
                elseif(boundh_y(k)>dim)
                    footprint_mat(1:boundh_x(k),boundl_y(k):dim,k)=footprint_mat(1:boundh_x(k),boundl_y(k):dim,k)+footprint(-boundl_x(k)+2:size(footprint,1),1:size(footprint,2)-(boundh_y(k)-dim),rot(j,k));
                else
                    footprint_mat(1:boundh_x(k),boundl_y(k):boundh_y(k),k)=footprint_mat(1:boundh_x(k),boundl_y(k):boundh_y(k),k)+footprint(-boundl_x(k)+2:size(footprint,1),:,rot(j,k));
                end
            elseif(boundh_x(k)>dim)
                if(boundl_y(k)<=0)
                    footprint_mat(boundl_x(k):dim,1:boundh_y(k),k)=footprint_mat(boundl_x(k):dim,1:boundh_y(k),k)+footprint(1:size(footprint,1)-(boundh_x(k)-dim),-boundl_y(k)+2:size(footprint,2),rot(j,k));
                elseif(boundh_y(k)>dim)
                    footprint_mat(boundl_x(k):dim,boundl_y(k):dim,k)= footprint_mat(boundl_x(k):dim,boundl_y(k):dim,k)+footprint(1:size(footprint,1)-(boundh_x(k)-dim),1:size(footprint,2)-(boundh_y(k)-dim),rot(j,k));
                else
                    footprint_mat(boundl_x(k):dim,boundl_y(k):boundh_y(k),k)= footprint_mat(boundl_x(k):dim,boundl_y(k):boundh_y(k),k)+footprint(1:size(footprint,1)-(boundh_x(k)-dim),:,rot(j,k));
                end
            elseif(boundh_y(k)>dim)
                footprint_mat(boundl_x(k):boundh_x(k),boundl_y(k):dim,k)=footprint_mat(boundl_x(k):boundh_x(k),boundl_y(k):dim,k)+footprint(:,1:size(footprint,2)-(boundh_y(k)-dim),rot(j,k));
            elseif(boundl_y(k)<=0)
                footprint_mat(boundl_x(k):boundh_x(k),1:boundh_y(k),k)= footprint_mat(boundl_x(k):boundh_x(k),1:boundh_y(k),k)+footprint(:,-boundl_y(k)+2:size(footprint,2),rot(j,k));
            else
                footprint_mat(boundl_x(k):boundh_x(k),boundl_y(k):boundh_y(k),k)=footprint_mat(boundl_x(k):boundh_x(k),boundl_y(k):boundh_y(k),k)+footprint(:,:,rot(j,k));
            end
        end
    end
    footprint_mat=reshape(footprint_mat,dim*dim,[]);
  
end

