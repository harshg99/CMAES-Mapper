function [j,k]=XYtoMat(origin,wx,wy,resolution)
    k=int32((wx-origin.x)/resolution);
    j=int32((wy-origin.y)/resolution);
end