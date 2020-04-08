function [footprints] = angularFootprints(sensor,simPar)
    j_c=sensor.range/simPar.resolution+1;
    k_c=sensor.range/simPar.resolution+1;
    angles=[0:1:359]-180;
    footprint_c=[0:(2*j_c-1)*(2*k_c-1)-1]';
    
    % Raycast of angles
    atan_footprint_c = atan2d(j_c-(mod(footprint_c,2*j_c-1)+1),(footprint_c/(2*k_c-1)+1)-k_c);
    
    
    footprint = (((footprint_c/(2*j_c-1)+1-j_c).^2+(mod(footprint_c,2*k_c-1)+1-k_c).^2)<=power(int32(sensor.range/simPar.resolution),2));
        % angle=abs(atan2d(j_c-(mod(footprint_c,2*j_c-1)+1),(footprint_c/(2*k_c-1)+1)-k_c));
    
    %toc;
    footprints = zeros(2*j_c-1,2*k_c-1,length(angles));
    
    for (j = 1: length(angles))
        x1 = footprint & (((atan_footprint_c <= (angles(j)+sensor.fov) & (atan_footprint_c >= angles(j))) & (angles(j)+sensor.fov <= 180)) | (((atan_footprint_c <= (angles(j)+sensor.fov - 360)) | ((atan_footprint_c <= 180) & (atan_footprint_c >= angles(j)))) & (angles(j)+sensor.fov > 180)));
        x2 = footprint & (((atan_footprint_c >= (angles(j)-sensor.fov) & (atan_footprint_c <= angles(j))) & (angles(j)-sensor.fov >=-180)) | (((atan_footprint_c >= (angles(j)-sensor.fov + 360)) | ((atan_footprint_c >=-180) & (atan_footprint_c <= angles(j)))) & (angles(j)-sensor.fov <-180)));
        x = x1 | x2;
        footprints(:,:,j)=reshape(x,2*j_c-1,2*k_c-1,1); 
    end
end
