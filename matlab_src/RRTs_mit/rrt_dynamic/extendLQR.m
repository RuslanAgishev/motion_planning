function [new_vert] = extendLQR(closest_vert,xy,K)

x_bar=-(xy-closest_vert);
u=-K*x_bar;

u=round(u,1);

if u>5
    u=5;
end
if u<-5
    u=-5;
end

%simulation pendulum
f = @(t,z) dynamics(closest_vert,u);
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