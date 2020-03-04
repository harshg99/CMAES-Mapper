function [traj]=toTraj(start,control,t_step,time)

% Generates a trajectory based on control
% Control arranged as (u_t1,u_theta1,u_t2,u_theta2,...)
% start configuration (x,y,theta)

%% Initialising trajectory
traj_.states=zeros(3,time/t_step+1);
traj_.num=1;
traj_.cost=0;
traj_.states(:,1)=start;

for k=1:size(control,1)
for j=1:(time/t_step)
   try
    x_t=traj_.states(:,j);
   catch
    j;
    size(traj_.states,2);
   end
    %x_t1=x_t+t_step*[control(k,2*j-1)*cos(x_t(3));control(k,2*j-1)*sin(x_t(3));control(k,2*j)];
    lambda=int32(ceil(j/(time/t_step)*size(control,2)/2));
    unit_control=[control(k,2*lambda-1);control(k,2*lambda)];
    x_t1(3)=x_t(3)+unit_control(2)*t_step;
    if(x_t1(3)>pi)
        x_t1(3)=x_t1(3)-(((floor(x_t1(3)/pi)-1)/2)+1)*2*pi;
    end
    
    if(x_t1(3)<-pi)
        x_t1(3)=x_t1(3)+(((floor(x_t1(3)/(-1*pi))-1)/2)+1)*2*pi;
    end
    
    if(abs(unit_control(2)-0.0)<0.0001)
        x_t1(1)=x_t(1)+t_step*unit_control(1)*cos(x_t(3));
        x_t1(2)=x_t(2)+t_step*unit_control(1)*sin(x_t(3));
    else
        x_t1(1)=x_t(1)+unit_control(1)/unit_control(2)*(sin(x_t1(3))-sin(x_t(3)));
        x_t1(2)=x_t(2)+unit_control(1)/unit_control(2)*(cos(x_t(3))-cos(x_t1(3)));
    end 
    traj_.states(:,j+1)=x_t1;
    traj_.num=traj_.num+1;
end
    traj(k)=traj_;
end

end