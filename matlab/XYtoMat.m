function [j,k]=XYtoMat(origin,wx,wy,resolution)
    j=floor((wx-origin.x)/resolution);
    k=floor((wy-origin.y)/resolution);
end