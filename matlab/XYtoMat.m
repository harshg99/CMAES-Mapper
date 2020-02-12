function [j,k]=XYtoMat(origin,wx,wy,resolution)
    k=floor((wx-origin.x)/resolution);
    j=floor((wy-origin.y)/resolution);
end