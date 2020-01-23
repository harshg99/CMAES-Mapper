function [wx,wy]=matToXY(origin,j,k,resolution)
    wx=origin.x+j*resolution;
    wy=origin.y+k*resolution;
end