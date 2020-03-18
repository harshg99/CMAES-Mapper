function plotTrajonMap(trajs,map,simPar,k)
map_colour=uint8(zeros(size(map,1),size(map,2),3));
map_colour(:,:,1)=uint8(map);
map_colour(:,:,2)=uint8(map);
map_colour(:,:,3)=uint8(map);
if(nargin==5)
    for(a=1:size(trajs(k).states,2))
        [p1,p2]=XYtoMat(simPar.origin,trajs(k).states(1,a),trajs(k).states(2,a),simPar.resolution);
        map_colour(p1,p2,1)=255;
        map_colour(p1,p2,2)=0;
        map_colour(p1,p2,3)=0;
    end
    figure();
    imshow(map_colour);
    return;
end
    for (j=1:size(trajs,2))
        for (a=1:size(trajs(j).states,2))
            [p1,p2]=XYtoMat(simPar.origin,trajs(j).states(1,a),trajs(j).states(2,a),simPar.resolution);
            map_colour(p1,p2,1)=255;
            map_colour(p1,p2,2)=0;
            map_colour(p1,p2,3)=0;
        end
    end
    figure();
    %map_colour(230,:,1)=255;
    map_colour=rot90(map_colour,1);
    imshow(map_colour);
end