clear all
close all

% ==================== Trajectory Generation for CyberShip II ====================
% This script simulates the open-loop motion of the CyberShip II vessel under
% piecewise-constant control, generating a reference trajectory for formation or
% tracking tasks. The result is visualized and saved as 'targettrajectory.mat'.
%
% Author: Wenxiang Wu
% Date:   2025-01-30


T=170;
Ts=1;
current=zeros(T,3);
dw=zeros(T,3);
dynamics   = createASV("Cybership2"); 
x0= [0,0,0,0,0,0]; % x y psi u v r initial state 
 

xi(1,:)=x0;
for i=1:T   
    disturbance.current=current(i,:);
    disturbance.force=dw(i,:);

    R2{i}=[
    cos(xi(i,3))  sin(xi(i,3)) 0
    -sin(xi(i,3)) cos(xi(i,3)) 0
          0             0      1
    ];
    xr(i,1) = xi(i,1)+current(i,1); 
    xr(i,2) = xi(i,2)+current(i,2);
    xr(i,3) = xi(i,3); 
    xr(i,4) = xi(i,4)+R2{i}(1,:)*current(i,:)';
    xr(i,5) = xi(i,5)+R2{i}(2,:)*current(i,:)'; 
    xr(i,6) = xi(i,6);
    if i<40
        tau(i,1) = 0.35;
        tau(i,2) = 0;
        tau(i,3) = 0;
    elseif i<150
        tau(i,1) = 0.35;
        tau(i,2) = -0.2;
        tau(i,3) = 0.02; 
    else
        tau(i,1) = 0.2;
        tau(i,2) = 0;
        tau(i,3) = 0; 
    end

    z =dynamicsModel(dynamics,xi(i,:)',tau(i,:)',disturbance);
        %%
    xi(i+1,1) = xi(i,1)+Ts*z(1,1);
    xi(i+1,2) = xi(i,2)+Ts*z(2,1); 
    xi(i+1,3) = xi(i,3)+Ts*z(3,1);
    xi(i+1,4) = xi(i,4)+Ts*z(4,1);
    xi(i+1,5) = xi(i,5)+Ts*z(5,1);
    xi(i+1,6) = xi(i,6)+Ts*z(6,1);      
end
for i=1:length(xr)-1
    if xr(i+1,3)-xr(i,3)<-pi
        xr(i+1,3)=xr(i+1,3)+2*pi;
    elseif xr(i+1,3)-xr(i,3)>pi
        xr(i+1,3)=xr(i+1,3)-2*pi;
    else
        xr(i+1,3)=xr(i+1,3);
    end
end

plot_num=T/Ts;
plot_step=10;
color1=[204, 0, 0]/255;
figure(1)
hold on
plot(xr(1:plot_num,2), xr(1:plot_num,1), '-','Color',[0, 0, 0]/255, 'LineWidth', 2);% actual trajectory  
for i=1:floor(plot_num/plot_step)
    ASVDisplay3([xr(1+(i-1)*(plot_step),3),0,0],xr(1+(i-1)*plot_step,2),xr(1+(i-1)*plot_step,1),[],[],color1);   
end
ASVDisplay3([xr(plot_num,3),0,0],xr(plot_num,2),xr(plot_num,1),[],[],color1);  
axis equal
ylim ( [-1 14]);

save('targettrajectory','xr')








