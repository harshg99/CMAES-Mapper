function [footprint_mat]=footprint(traj,sensor,dim,origin,resolution)
%% Function evaluates the footprint of the robot
% traj:  trajectory structure
% sensor: sensor information
% dim : dimension of the map
% origin: origin of the map
% resolution: map resolution
%%


footprint_mat=zeros(dim);
size(traj.states,2);
for(j=1:size(traj.states,2))
    mat=zeros(dim);
    state=traj.states(:,j);
    
    [m,n]=XYtoMat(origin,state(1),state(2),resolution);
    
    footprint=zeros(2*sensor.range/resolution+1);
    j_c=sensor.range/resolution;
    k_c=sensor.range/resolution;
    for(a=1:size(footprint,1))
        for(b=1:size(footprint,2))
            if(sqrt((a-j_c)^2+(b-k_c)^2)<sensor.range/resolution)
                if(abs((atan2d(j_c-a,k_c-b)-state(3)*180/pi))<=sensor.fov)
                    footprint(a,b)=1;
                end
            end
        end
    end
    boundl_x=m-int32(sensor.range/resolution);
    boundh_x=m+int32(sensor.range/resolution);
    
    boundl_y=n-int32(sensor.range/resolution);
    boundh_y=n+int32(sensor.range/resolution);
    
    if(boundl_x<=0)
        if(boundl_y<=0)
            mat(1:boundh_x,1:boundh_y)=footprint(-boundl_x+2:size(footprint,1),-boundl_y+2:size(footprint,2));
        elseif(boundh_y>dim)
            mat(1:boundh_x,boundl_y:dim)=footprint(-boundl_x+2:size(footprint,1),1:size(footprint,2)-(boundh_y-dim));    
        else
            mat(1:boundh_x,boundl_y:boundh_y)=footprint(-boundl_x+2:size(footprint,1),:);
        end
    elseif(boundh_x>dim)
        if(boundl_y<=0)
            mat(boundl_x:dim,1:boundh_y)=footprint(1:size(footprint,1)-(boundh_x-dim),-boundl_y+2:size(footprint,2));
        elseif(boundh_y>dim)
            mat(boundl_x:dim,boundl_y:dim)=footprint(1:size(footprint,1)-(boundh_x-dim),1:size(footprint,2)-(boundh_y-dim));    
        else
            mat(boundl_x:dim,boundl_y:boundh_y)=footprint(1:size(footprint,1)-(boundh_x-dim),:);
        end
    elseif(boundh_y>dim)
            mat(boundl_x:boundh_x,boundl_y:dim)=footprint(:,1:size(footprint,2)-(boundh_y-dim));
    elseif(boundl_y<=0)
            mat(boundl_x:boundh_x,1:boundh_y)=footprint(:,-boundl_y+2:size(footprint,2));
    else
            mat(boundl_x:boundh_x,boundl_y:boundh_y)=footprint;
    end
            footprint_mat=footprint_mat+mat;

end
max(max(footprint_mat));
end