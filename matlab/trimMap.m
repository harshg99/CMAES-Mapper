function [map,infomap,costmap]=trimMap(map_,infomap_,costmap_)

%% Reads and stores map files
%------------------------------------
% file1 : name of map file
% file2 : name of informaton overlay file
% file3 : name of costmap file
%%

map=map_;
infomap=infomap_;
costmap=costmap_;

%map=map_(size(map_,1)/2-range/resolution:size(map_,1)/2+range/resolution-1,size(map_,2)/2-range/resolution:size(map_,2)/2+range/resolution-1);
%infomap=infomap_(size(map_,1)/2-range/resolution:size(map_,1)/2+range/resolution-1,size(map_,2)/2-range/resolution:size(map_,2)/2+range/resolution-1);
%costmap=costmap_(size(map_,1)/2-range/resolution:size(map_,1)/2+range/resolution-1,size(map_,2)/2-range/resolution:size(map_,2)/2+range/resolution-1);
infomap = 255 - infomap;
costmap = 255 - costmap;
map(map == 255) = 205;
map=rot90(map,3);
infomap=rot90(infomap,3);
costmap=rot90(costmap,3);


% for(j=1:size(map_,1))
%     for(k=1:size(map_,2))
%         wx=origin.x+j*resolution;
%         wy=origin.y+k*resolution;
%         if((abs(wx-c_x)>=range) || (abs(wy-c_y)>=range))
%             map(j,k)=0;
%             costmap(j,k)=255;
%             infomap(j,k)=0;
%         end
%     end
% end

end