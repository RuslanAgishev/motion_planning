function dX = dynamics(closest_vert,u0)

b=.1;
g=9.81;

th_d=closest_vert(2);
th_dd=u0-b*closest_vert(2)-g*sin(closest_vert(1));

dX=[th_d;th_dd];

end