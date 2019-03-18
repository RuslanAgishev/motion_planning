function [closest_vert] = closestVertexEuclidean(rrt_verts,xy)

% rrt_verts=[-1 4 1 0 3  8
%             4 5 9 7 6 7];    
% xy=[4
%     -3];

distance=zeros(1,length(rrt_verts(1,:)));
for k = 1:length(rrt_verts(1,:))
    
    xy_wr=wrapToPi(xy(1)+pi/2);
    rrt_verts_wr=wrapToPi(rrt_verts(1,k)+pi/2);
    th=xy_wr-rrt_verts_wr;
    
    th_d=(xy(2)-rrt_verts(2,k));
    distance(k)=sqrt(th^2+th_d^2);
end

distance

[ind, ind]=min(distance);

closest_vert=rrt_verts(:,ind);


end