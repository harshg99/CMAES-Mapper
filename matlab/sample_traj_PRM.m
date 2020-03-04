function [trajs]=sample_traj_PRM(map,simPar)

% Samples a trajectory parametrised on control( control tuples

% origin: Origin for the map
% GMDist: gm distribution for sampling
% simPar: simulation variables
%costmap for checking trajectory

a = binaryOccupancyMap(map,1/simPar.resolution);
a.GridLocationInWorld=[simPar.origin.x,simPar.origin.y];
rm = PRM(a,simPar.numNodes);
rm.ConnectionDistance=simPar.connectionDistance;
rm.GMDist = simPar.GMDist;
update(rm);
roadmap=rm.RoadMap.Roadmap;
toc;

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


while (i<=simPar.numTraj)
    isVisited = zeros(1,length(roadmap.NodeList));
    j=0;
    current = newNodeID;
    trajectories = zeros(2,simPar.depth + 1);
    while j<simPar.depth
        neighbours = roadmap.EdgeList(:,roadmap.EdgeList(2,:)==current);
        neighbours = neighbours(isVisited(neigbours));
        if(isempty(neighbours))
            current = trajectories(:,j-1);
        else
            current = neighbours(int32(rand(1,1)* length(neighbours)));
        end
        
        trajectories(:,j+1) = current;
        isVisited(current)=1;
        j=j+1;
    end
    i=i+1;
    trajs(i)=toTrajPRM(trajectory,simPar);
end
end