function [footprint_mat]=footprint3(traj,sensor,dim,origin,resolution)
%% Function evaluates the footprint of the robot
% traj:  trajectory structure
% sensor: sensor information
% dim : dimension of the map
% origin: origin of the map
% resolution: map resolution
%%
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
   
    boundl_x=m-sensor.range/resolution;
    boundh_x=m+sensor.range/resolution;
    
    boundl_y=n-sensor.range/resolution;
    boundh_y=n+sensor.range/resolution;
    
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
max(max(footprint_mat));

end

