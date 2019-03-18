function [closest_vert] = closestVertex(rrt_verts,xy)


% xy=[0;0];
% rrt_verts=[6 8 7 4 8 9
%            4 5 8 6 1 7];

distance=zeros(1,length(rrt_verts(1,:)));
for k = 1:length(rrt_verts(1,:))
    distance(k)=sqrt((xy(1)-rrt_verts(1,k))^2+(xy(2)-rrt_verts(2,k))^2);
end

[dmin, ind]=min(distance);

closest_vert=rrt_verts(:,ind);


end

