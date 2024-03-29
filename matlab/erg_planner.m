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
function [optimal_trajectory]= erg_planner(mode,map,infomap,costmap,varargin)

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
    simPar.m_prim=5;
    
    mean=zeros(2*simPar.m_prim,1);
    variance=3*eye(2*simPar.m_prim);
    
    mean = zeros(simPar.m_prim,1);
    variance = 0.5*eye(simPar.m_prim);     
    velocity = 0.5;
    
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
    
    [map_Data,infomap_Data,costmap_Data]=readFile('maps/map3.pgm','maps/infomap3.pgm','maps/costmap3.pgm');
    [map_Data,infomap_Data,costmap_Data]=trimMap(map_Data,infomap_Data,costmap_Data);
    
    if(mode=="PRM")
        map = map_Data;
        map_Data(map_Data==205)=0;
        map_Data = 255 - map_Data;
        map_Data(map_Data>1) = 0;
        map_Data = 1 - map_Data;
        map_Data=rot90(map_Data,1);
    end
    
    resolution = 0.05;
    param = ReadYaml("maps/map3.yaml");
    
    origin.x = param.origin(1);
    origin.y = param.origin(2);
    %% Simulation Initialise Size
    simPar.t_step=0.1;
    simPar.sensor.sensor_rate=1.0;
    simPar.sensor.range = 5;
    simPar.sensor.fov=60;
    simPar.time=40;
    simPar.stages=10;
    simPar.m_prim=8;
    
    mean=zeros(2*simPar.m_prim,1);
    variance= 3 * eye(2*simPar.m_prim);
%     
    mean = zeros(simPar.m_prim,1);
    variance = 3*eye(simPar.m_prim);     
    velocity = 1.0;
    
    simPar.start=[0;0;0];
    simPar.origin=origin;
    simPar.resolution=resolution;
    simPar.numTraj = 100;
    simPar.percentile=0.1;
    simPar.cutoff = 0.0001;
    if(mode =="PRM")
        simPar.GMDist = 0;
        simPar.numNodes =200;
        simPar.stages = 10;
        simPar.step = 0.5;
        simPar.numTraj = 100;
        simPar.percentile=0.01;
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
optimal_trajectory = 0;
f = angularFootprints(simPar.sensor,simPar);

%% CE Optimisation
for (i=1:simPar.stages)
    if(mode=="control")
        %[traj,control]=sample_traj(mean,variance,simPar,costmap_Data);
        [traj,control]=sample_traj(mean,variance,simPar,costmap_Data,velocity);   
        %[footprint_mat]=footprint3(traj,simPar.sensor,size(map_Data),simPar,simPar.sensor.sensor_rate/simPar.t_step);
        [footprint_mat]=footprint4(traj,simPar.sensor,size(map_Data),simPar,simPar.sensor.sensor_rate/simPar.t_step,f,90);
        plotTrajonMap(traj,map_Data,simPar);
    elseif(mode=="PRM")
        a = binaryOccupancyMap(map_Data,1/simPar.resolution);
        a.GridLocationInWorld=[simPar.origin.x,simPar.origin.y];
        rm = PRM(a,simPar.numNodes);
        rm.ConnectionDistance=simPar.connectionDistance;
        rm.GMDist = simPar.GMDist;
        update(rm);
        [traj]=sample_traj_PRM(rm,simPar,costmap_Data);
        [footprint_mat]=footprint4(traj,simPar.sensor,size(map_Data),simPar,1,f,90);
        plotTrajonMap(traj,map,simPar);
    end
    
    footprint_mat=footprint_mat./max(footprint_mat,[],1);
    costs=0.001*[traj.cost]+ KLDiv(footprint_mat,infomap_Data(:)-1) + 0.005 * [traj.length];%+0.00*sum(control(k,:).*control(k,:))*t_step;
    % if(cost<min_cost)
    %     min_cost=cost;
    % end
    % if(cost>max_cost)
    %     max_cost=cost;
    % end
    
    sort_costs=sort(costs);
    min_cost(i)=sort_costs(1);
    thresh= sort_costs(int32(simPar.percentile*simPar.numTraj));
    if(mode == "control") 
        vec=zeros(simPar.percentile*simPar.numTraj,size(control,2));
        a=1;
        for(m=1:size(traj,2))
            if(costs(m)<=thresh & a<=simPar.percentile*simPar.numTraj)
                vec(a,:)=control(m,:);
                a=a+1;
            end
        end
        [mean,variance]=getMeanVariance(vec);
    elseif (mode == "PRM")
        vec = [];
        a=1;
        for(m=1:size(traj,2))
            if(costs(m)<=thresh & a<=simPar.percentile*simPar.numTraj)
                vec=[vec,traj(m).states(1:2,:)];
                a=a+1;
            end
        end
        %vec(1,:) = vec(1,:) / (rm.Map.XWorldLimits(2)-rm.Map.XWorldLimits(1) );
        %vec(2,:) = vec(2,:) / (rm.Map.YWorldLimits(2)-rm.Map.YWorldLimits(1) );
        min_optimal_clusters = 1 ;
        max_optimal_clusters = simPar.depth ; 
        while (min_optimal_clusters<max_optimal_clusters)
            try
                simPar.GMDist = fitgmdist(vec',ceil((max_optimal_clusters+min_optimal_clusters)/2),'Start','plus');
                min_optimal_clusters=ceil((max_optimal_clusters+min_optimal_clusters)/2);
            catch
                max_optimal_clusters=floor((max_optimal_clusters+min_optimal_clusters)/2);
            end
        end
%           figure();
%           
%           simPar.GMDist = fitgmdist(vec',ceil(5),'Start','plus');
%           gmPDF = @(x,y) arrayfun(@(x0,y0) pdf(simPar.GMDist,[x0,y0]),x,y);
%           fcontour(gmPDF,[-12,12]);
    end
    optimal = traj(find(costs == min_cost(i)));
    
    if(i~=1)
        if(abs((min_cost(i)-min_cost(i-1))/min_cost(i))<simPar.cutoff)
                optimal_trajectory = optimal;
                %break;
        elseif(min_cost(i)<=1.0001 * min_cost(i-1))
            optimal_trajectory = optimal;
        else
             if(i > 6)
                 break;
             end
        end
    else
        optimal_trajectory = optimal;
    end
    
%plotTrajonMap(traj,map,origin2,resolution)
toc;
end
toc;
%[footprint_mat]=footprint(traj(2),sensor,size(costmap,1),origin2,resolution);
plotTrajonMap(optimal_trajectory,map,simPar);
%footprint_mat=footprint_mat/max(max(footprint_mat));
figure();
fp = footprint4(optimal_trajectory,simPar.sensor,size(map_Data),simPar,1,f,90);
fp = fp/max(fp);
imshow(reshape(fp,size(map_Data)),[0 1]);
end