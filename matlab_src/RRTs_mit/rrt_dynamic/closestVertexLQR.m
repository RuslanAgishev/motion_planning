function [closest_vert,K] = closestVertexLQR(rrt_verts,xy)

% rrt_verts=[0 1 2 2 2 1
%            0 0 0 2 4 5];    
% xy=[1
%     2];

g=9.81;
b=.1;
B=[0
   1];
Q=eye(2);
R=.1;

xy(1)=wrapToPi(xy(1)+pi/2);
distance=zeros(1,length(rrt_verts(1,:)));
for k = 1:length(rrt_verts(1,:))
    %state of the pendulum
    x0=rrt_verts(:,k);
    x0(1)=wrapToPi(x0(1)+pi/2);
    x_bar=xy-x0;
    A=[0                1
       -g*cos(x0(1))   -b];
   [K,S] = lqr(A,B,Q,R);
   distance(k)=x_bar'*S*x_bar;
end
distance;
[ind ind]=min(distance);

closest_vert=rrt_verts(:,ind);
A=[0                         1
  -g*cos(closest_vert(1))   -b];
[K,S] = lqr(A,B,Q,R);

%closest_vert
end


% %state of the pendulum
% x0=[pi-.1
%     2];
% u0=0;
% 
% %sample point
% x=[pi+.2
%    -5];
% 
% x_bar=x-x0
% 
% g=9.81;
% b=.1;
% 
% A=[0                1
%    -g*cos(x0(1))   -b];
% 
% B=[0
%    1];
% 
% Q=eye(2);
% R=.1;
% [K,S] = lqr(A,B,Q,R)
% 
% metric=x_bar'*S*x_bar