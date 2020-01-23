%% Main code for ergodic sampling
clear all;
close all;

%%Reading Map
[map,infomap,costmap]=readFile('map.pgm.pgm','infomap.pgm.pgm','costmap.pgm');
resolution=0.05;
origin.x=-30;
origin.y=-30;

% Show the files
figure();
imshow(map);

figure();
imshow(infomap);

figure();
imshow(costmap);

range=12;
origin2.x=-1*range;
origin2.y=-1*range;
[map,infomap,costmap]=trimMap(map,infomap,costmap,range,origin,resolution);


figure();
imshow(map);

figure();
imshow(infomap);

figure();
imshow(costmap);

%% Simulation Initialise Size
t_step=0.1;
time=60;
stages=10;
m_prim=10;
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
[traj,control]=sample_traj(mean,variance,samples,simPar,costmap);

plotTraj(traj,range);

%% Check footprint
sensor.range=3;
sensor.fov=60;
dim=2*range/resolution;
min_cost=inf;
max_cost=-inf;
costs=zeros(size(traj,2),1);
min_cost=zeros(stages,1);


%% CE Optimisation
for(i=1:stages)
    
for(k=1:size(traj,2))
[footprint_mat]=footprint(traj(k),sensor,dim,origin2,resolution);
footprint_mat=footprint_mat/max(max(footprint_mat));
traj(k).cost=traj(k).cost+KLDiv(footprint_mat,infomap)+0.00*sum(control(k,:).*control(k,:))*t_step;
% if(cost<min_cost)
%     min_cost=cost;
% end
% if(cost>max_cost)
%     max_cost=cost;
% end
costs(k)=traj(k).cost;
end
costs=sort(costs);
min_cost(i)=costs(1);
thresh= costs(int32(percentile*samples));
vec=zeros(percentile*samples,2*m_prim);
a=1;
for(m=1:size(traj,2))
    if(traj(m).cost<=thresh & a<=percentile*samples)
        vec(a,:)=control(m,:);
        a=a+1;
    end
end
a
[mean,variance]=getMeanVariance(vec);
[traj,control]=sample_traj(mean,variance,samples,simPar,costmap);     
plotTrajonMap(traj,map,origin2,resolution)
end
[footprint_mat]=footprint(traj(2),sensor,dim,origin2,resolution);
plotTraj(traj,range,2);
footprint_mat=footprint_mat/max(max(footprint_mat));
imshow(footprint_mat);
