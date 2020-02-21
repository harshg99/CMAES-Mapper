%% Main code for ergodic sampling
rosinit();
map_sub = rossubscriber('/robot1/map');
infomap_sub = rossubscriber('/robot1/infomap');
costmap_sub = rossubscriber('/robot1/move_base/global_costmap/costmap');

traj=planner(map_sub.LatestMessage,infomap_sub.LatestMessage,costmap_sub.LatestMessage);
rosshutdown();
