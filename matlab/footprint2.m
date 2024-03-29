function [footprint_mat]=footprint2(traj,sensor,dim,origin,resolution,sample_fac)


%% Function evaluates the footprint of the robot
% traj:  trajectory structure
% sensor: sensor information
% dim : dimension of the map
% origin: origin of the map
% resolution: map resolution
% sample_fac: How many trajectory staes would a sensor receive information
%%

footprint_mat=repmat((0:dim*dim-1),size(traj.states,2),1);
trajs_x=repmat(traj.states(1,:)',1,dim*dim);
trajs_y=repmat(traj.states(2,:)',1,dim*dim);
theta=  repmat(traj.states(3,:)',1,dim*dim)*180/pi;
footprint_mat = ((((((footprint_mat/dim)+1)*resolution+origin.x)-trajs_x).^2+(((mod(footprint_mat,dim)+1)*resolution+origin.y)-trajs_y).^2)<=9);
footprint_mat = footprint_mat & (abs(((atan2d((((footprint_mat/dim)+1)*resolution+origin.x)-trajs_x,(((mod(footprint_mat,dim)+1)*resolution+origin.y)-trajs_y))-theta)))<=sensor.fov);
footprint_mat= sum(footprint_mat,1); 
end