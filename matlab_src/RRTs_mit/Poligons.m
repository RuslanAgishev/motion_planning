xi = randn(2,10); % obstacle
x0 = randn(2,10) ;% given set of points, to check intersection with xi


k = convhull(xi(1,:),xi(2,:));
x=xi(1,:);
y=xi(2,:);
figure (1)
plot(x(k),y(k),'r-',x,y,'b*')
grid on


xq=x0(1,:);
yq=x0(2,:);
[in,on] = inpolygon(xq,yq,x(k),y(k));

figure (2)

plot(x(k),y(k),'r-') % polygon
axis equal

hold on
plot(xq(in),yq(in),'r+') % points inside
plot(xq(~in),yq(~in),'bo') % points outside
grid on
hold off

%points_out=numel(xq(~in))

collisionFree = ~in;