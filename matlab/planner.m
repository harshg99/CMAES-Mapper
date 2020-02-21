%% Main code for ergodic sampling

%% Ergodic planner

% Inputs
%---------------------------------------------------
% map: Input map message (nav_msgs/OccupancyGrid)
% infomap: INformation map (nav_msgs/OccupancyGrid)
% costmap: Costmap (nav_msgs/OccupancyGrid)
function [traj]= erg_planner(map,infomap,costmap)

clear all;
close all;



%% Reshaping data in map

map_Data=reshape(map.Data,map.Info.Height,map.Info.Width);

[map,infomap,costmap]=readFile('map.pgm.pgm','infomap.pgm.pgm','costmap.pgm');
resolution=0.05;
origin.x=-30;
origin.y=-30;

% Show the files
% figure();
% imshow(map);
%
% figure();
% imshow(infomap);
%
% figure();
% imshow(costmap);

range=12;
origin2.x=-1*range;
origin2.y=-1*range;
[map,infomap,costmap]=trimMap(map,infomap,costmap,range,origin,resolution);

costmap(map==205)=255;
map2=(costmap>200);

%
% figure();
% imshow(map);
%
% figure();
% imshow(infomap);
%
% figure();(
% imshow(costmap);

%% Simulation Initialise Size
t_step=0.1;
sensor_rate=1.0;
time=40;
stages=10;
m_prim=12;
mean=zeros(2*m_prim,1);
variance=3*eye(2*m_prim);
samples=50;
start=[0;0;0];
percentile=0.1;
simPar.time=time;
simPar.t_step=t_step;
simPar.start=start;
simPar.origin=origin2;
simPar.resolution=resolution;

%% Initial Samples
%[traj,control]=sample_traj(mean,variance,samples,simPar,costmap);

%plotTraj(traj,range);
%% Check footprint
sensor.range=3;
sensor.fov=60;
dim=2*range/resolution;
min_cost=inf;
max_cost=-inf;
costs=zeros(samples,1);
min_cost=zeros(stages,1);
sort_costs=zeros(samples,1);

tic;

%% CE Optimisation
for(i=1:stages)
    tic;
    [traj,control]=sample_traj(mean,variance,samples,simPar,costmap);
    [footprint_mat]=footprint3(traj,sensor,size(map,1),origin2,resolution,sensor_rate/t_step);
    plotTrajonMap(traj,map,origin2,resolution);
    footprint_mat=footprint_mat./max(footprint_mat,[],1);
    costs=[traj.cost]+KLDiv(footprint_mat,infomap(:));%+0.00*sum(control(k,:).*control(k,:))*t_step;
    toc;
    % if(cost<min_cost)
    %     min_cost=cost;
    % end
    % if(cost>max_cost)
    %     max_cost=cost;
    % end
    
    sort_costs=sort(costs);
    min_cost(i)=sort_costs(1);
    thresh= sort_costs(int32(percentile*samples));
    vec=zeros(percentile*samples,2*m_prim);
    a=1;
    for(m=1:size(traj,2))
        if(costs(m)<=thresh & a<=percentile*samples)
            vec(a,:)=control(m,:);
            a=a+1;
        end
    end
end
[mean,variance]=getMeanVariance(vec);
if(i~=1)
    if(abs((min_cost(i)-min_cost(i-1))/min_cost(i))<0.01)
        break;
    end
end
%plotTrajonMap(traj,map,origin2,resolution)

end
toc;
%[footprint_mat]=footprint(traj(2),sensor,size(costmap,1),origin2,resolution);
plotTrajonMap(traj,map,origin2,resolution,10);
%footprint_mat=footprint_mat/max(max(footprint_mat));
figure();
imshow(reshape(footprint_mat(:,10),480,480),[0 1]);
end