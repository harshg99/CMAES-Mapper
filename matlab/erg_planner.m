%% Main code for ergodic sampling

%% Ergodic planner

% Inputs
%---------------------------------------------------
% map: Input map message (nav_msgs/OccupancyGrid)
% infomap: INformation map (nav_msgs/OccupancyGrid)
% costmap: Costmap (nav_msgs/OccupancyGrid)
% Varargin: 'time'- time for planning
%           't_step' - time for simulation 
%           'm_prim' - no.of motion primitives
%           'start' - start position
%           'samples' - no.of samples
%           'sensor' - sensor paramaters [sensor.rate ,sensor,fov,
%           sensor.range]

%
function [traj]= erg_planner(mode,map,infomap,costmap,varargin)

%% Reshaping data in map

if(nargin>1)
    
    map_Data=reshape(map.Data,map.Info.Width,map.Info.Height);
    infomap_Data=reshape(infomap.Data,infomap.Info.Width,infomap.Info.Height);
    costmap_Data=reshape(costmap.Data,costmap.Info.Width,costmap.Info.Height);
    %
    costmap_Data(map_Data==-1)=254;
    resolution=map.Info.Resolution;
    origin.x=map.Info.Origin.Position.X;
    origin.y=map.Info.Origin.Position.Y;
    %% Simulation Initialise Size
    simPar.t_step=0.1;
    simPar.sensor.sensor_rate=1.0;
    simPar.sensor.range = 3;
    simPar.sensor.fov=60;
    simPar.time=40;
    simPar.stages=10;
    simPar.m_prim=10;
    
    mean=zeros(2*simPar.m_prim,1);
    variance=3*eye(2*simPar.m_prim);

 
    simPar.time=time;
    simPar.t_step=t_step;
    simPar.start=[0;0;0];
    simPar.origin=origin;
    simPar.resolution=resolution;
    simPar.numTraj = 50;
    simPar.percentile=0.1;
    simPar.cutoff = 0.01;
    if(mode =="PRM")
        simPar.GMDist = 0;
        simPar.numNodes = 200;
        simPar.step = 0.5;
        simPar.depth = 10;
        simPar.connectionDistance = 10;
    end
else
    
    [map_Data,infomap_Data,costmap_Data]=readFile('map.pgm.pgm','infomap.pgm.pgm','costmap.pgm');
    resolution = 0.05;
    origin.x=-30;
    origin.y=-30;
    range=12;
    
    [map_Data,infomap_Data,costmap_Data]=trimMap(map_Data,infomap_Data,costmap_Data,range,origin,resolution);
    if(mode=="PRM")
        costmap_Data(map_Data==205)=254;
        costmap_Data(costmap_Data<200)=0;
    end
    
    origin.x=-1*range;
    origin.y=-1*range;
    
    %% Simulation Initialise Size
    simPar.t_step=0.1;
    simPar.sensor.sensor_rate=1.0;
    simPar.sensor.range = 3;
    simPar.sensor.fov=60;
    simPar.time=40;
    simPar.stages=10;
    simPar.m_prim=10;
    
    mean=zeros(2*simPar.m_prim,1);
    variance=3*eye(2*simPar.m_prim);
    
    simPar.start=[0;0;0];
    simPar.origin=origin;
    simPar.resolution=resolution;
    simPar.numTraj = 50;
    simPar.percentile=0.1;
    simPar.cutoff = 0.01;
    if(mode =="PRM")
        simPar.GMDist = 0;
        simPar.numNodes =200;
        simPar.step = 0.5;
        simPar.depth = 10;
        simPar.connectionDistance = 10;
    end
end



% Show the files
figure();
imshow(map_Data);

figure();
imshow(infomap_Data);

figure();
imshow(costmap_Data);



%
% figure();
% imshow(map);
%
% figure();
% imshow(infomap);
%
% figure();(
% imshow(costmap);



%% Initial Samples
%[traj,control]=sample_traj(mean,variance,samples,simPar,costmap);

%plotTraj(traj,range);
%% Check footprint

min_cost=inf;
max_cost=-inf;
costs=zeros(simPar.numTraj,1);
min_cost=zeros(simPar.stages,1);
sort_costs=zeros(simPar.numTraj,1);

tic;

%% CE Optimisation
for (i=1:simPar.stages)
    if(mode=="control")
        [traj,control]=sample_traj(mean,variance,simPar,costmap_Data);
        [footprint_mat]=footprint3(traj,simPar.sensor,size(map_Data),simPar,simPar.sensor.sensor_rate/simPar.t_step);
    elseif(mode=="PRM")
         [traj]=sample_traj_PRM(costmap_Data,simPar);
         [footprint_mat]=footprint3(traj,simPar.sensor,size(map_Data),simPar,1);
    end
    
    plotTrajonMap(traj,map_Data,simPar);
    footprint_mat=footprint_mat./max(footprint_mat,[],1);
    costs=[traj.cost]+KLDiv(footprint_mat,infomap_Data(:));%+0.00*sum(control(k,:).*control(k,:))*t_step;
    % if(cost<min_cost)
    %     min_cost=cost;
    % end
    % if(cost>max_cost)
    %     max_cost=cost;
    % end
    
    sort_costs=sort(costs);
    min_cost(i)=sort_costs(1);
    thresh= sort_costs(int32(simPar.percentile*simPar.numTraj));
    vec=zeros(simPar.percentile*simPar.numTraj,2*simPar.m_prim);
    a=1;
    
    for(m=1:size(traj,2))
        if(costs(m)<=thresh & a<=simPar.percentile*simPar.numTraj)
            vec(a,:)=control(m,:);
            a=a+1;
        end
    end
    [mean,variance]=getMeanVariance(vec);
    if(i~=1)
        if(abs((min_cost(i)-min_cost(i-1))/min_cost(i))<simPar.cutoff)
            break;
        end
    end
%plotTrajonMap(traj,map,origin2,resolution)
toc;
end
toc;
%[footprint_mat]=footprint(traj(2),sensor,size(costmap,1),origin2,resolution);
plotTrajonMap(traj,map_Data,simPar,10);
%footprint_mat=footprint_mat/max(max(footprint_mat));
figure();
imshow(reshape(footprint_mat(:,10),480,480),[0 1]);
end