function plotTraj(trajs,range,k)
figure();
axis([-range range -range range]);
set(gca,'Ydir','reverse');
if(nargin==3)
    plot(trajs(k).states(1,:),trajs(k).states(2,:),'-');
    axis([-range range -range range]);
    set(gca,'Ydir','reverse');
    return;
end
    for(j=1:size(trajs,2))
        plot(trajs(j).states(1,:),trajs(j).states(2,:),'-');
        hold on;
    end
    axis([-range range -range range]);
    set(gca,'Ydir','reverse');
    hold off;
end