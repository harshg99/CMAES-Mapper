function [trajs]=sample_traj_PRM(rm,simPar,costmap)

% Samples a trajectory parametrised on control( control tuples

% origin: Origin for the map
% GMDist: gm distribution for sampling
% simPar: simulation variables
%costmap for checking trajectory

roadmap=rm.RoadMap.Roadmap;
a=rm.Map;
% Adding robot location to roadmap
newNode = simPar.start(1:2);
newNodeID = roadmap.addNode(newNode);
[dist,nearByNodes] = roadmap.distanceFromAllNodes(newNode);

for i=1:length(dist)
    % If the same node then skip adding edge
    if nearByNodes(i) == newNodeID
        continue;
    end
    
    % If distance is greater than threshold then skip adding edge
    if dist(i) > simPar.connectionDistance
        break;
    end
    
    % If the line passes through obstacles then skip adding edge
    nearbyNode = roadmap.nodeCoordinate(nearByNodes(i));
    
    isObstacleFree = nav.algs.internal.raycast(newNode', ...
        nearbyNode', a.occupancyMatrix, a.Resolution,a.GridLocationInWorld);
    
    if ~isObstacleFree
        continue;
    end
    
    % Add an edge
    roadmap.addEdge(newNodeID, nearByNodes(i), dist(i));
end
% 
% trajs.states=cell(1,simPar.depth);
% trajs.num=1;
% trajs.cost=0;
i=0;

while (i<=simPar.numTraj)
    isVisited = zeros(1,length(roadmap.NodeList));
    j=1;
    current = newNodeID;
    prev_queue = [];
    
    trajectories = zeros(2,simPar.depth + 1);
    while j<=simPar.depth
        neighbours = roadmap.EdgeList(2,roadmap.EdgeList(1,:)==current);
        neighbours = neighbours(isVisited(neighbours)==0);
        if(isempty(neighbours))
            current=prev_queue(length(prev_queue));
            prev_queue(length(prev_queue))=[];
        else
            prev_queue(length(prev_queue)+1)=current;
            current = neighbours(int32(ceil((rand(1,1)* length(neighbours)))));
        end
        trajectories(:,j+1) = roadmap.NodeList(:,current);
        isVisited(current)=1;
        j=j+1;
    end
    i=i+1;
    %trajectories=[0 -1;1 0]*trajectories;
    trajs(i)=toTrajPRM(trajectories,simPar,costmap);
end
end