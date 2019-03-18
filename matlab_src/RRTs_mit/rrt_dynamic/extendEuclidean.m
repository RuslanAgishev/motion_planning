function [new_vert] = extendEuclidean(closest_vert,xy)

% closest_vert=[0
%               0];
% xy=[1.2
%     -4];

b=.1;
g=9.81;

%finding u0
th_d=closest_vert(2);
th_dd=-b*closest_vert(2)-g*sin(closest_vert(1));
tan1=th_dd/th_d;

if (closest_vert(1)==0)&&(closest_vert(2)==0)
    u0=xy(2);
end
if (xy(1)>closest_vert(1)&&closest_vert(2)>0) || (closest_vert(1)>xy(1)&&closest_vert(2)<0);
tan2=(xy(2)-closest_vert(2))/(xy(1)-closest_vert(1));
th_dd_new=tan2*th_d;
u0=tan2*th_d-th_dd;
else
u0=xy(2)-(closest_vert(2)+th_dd);
end

u0=round(u0,1);

if u0>5
    u0=5;
end
if u0<-5
    u0=-5;
end

u0

%simulation pendulum
f = @(t,z) dynamics(closest_vert,u0);
X0=[closest_vert(1);closest_vert(2)];
sol = ode45(f, [0 0.1], X0);

new_vert=sol.y(:,end);

if new_vert(1)>1.5*pi
    over=new_vert(1)-1.5*pi;
    new_vert(1)=-pi/2+over;
end
if new_vert(1)<-pi/2
    over=new_vert(1)-pi/2;
    new_vert(1)=1.5*pi+over;
end

%new_vert



end




% % Bounds on world
% world_bounds_th = [-pi/2,(3/2)*pi];
% world_bounds_thdot = [-10,10];
%  
% % Start and goal positions
% figure(1); clf;
% xy_start = [0;0]; plot(xy_start(1),xy_start(2),'bo','MarkerFaceColor','b','MarkerSize',10);
% xy_goal = [pi;0]; plot(xy_goal(1),xy_goal(2),'go','MarkerFaceColor','g','MarkerSize',10); drawnow;
% 
% figure(1); hold on;
% axis([world_bounds_th, world_bounds_thdot]);
%  hxy = plot(0,0,'ro');
%  grid on
%  hold on
% 
% x1=[closest_vert(1); closest_vert(1)+th_d];
% y1=[closest_vert(2); closest_vert(2)+th_dd];
% 
% plot(xy(1),xy(2),'bo',closest_vert(1),closest_vert(2),'go');
% plot(x1,y1,'r-');
% hold on
% 
% % x2=[closest_vert(1); new_vert(1)];
% % y2=[closest_vert(2); new_vert(2)];
% % plot(x1,y1,'g-');

