function [map,infomap,costmap]=readFile(file1,file2,file3)

%% Reads and stores map files
%------------------------------------
% file1 : name of map file
% file2 : name of informaton overlay file
% file3 : name of costmap file
%%
map=imread(file1);
infomap=imread(file2);
costmap=imread(file3);


end