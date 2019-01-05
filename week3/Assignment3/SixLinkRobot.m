function fv = SixLinkRobot (cspace)
% Simulate a simple six link revolute chain

l = 5;
h = 0.5;

link = boxFV(0,l,-h,h);

fv = link;

for i = 1:5
    fv = appendFV(link, transformFV(fv, cspace(i), [l 0]));
end

fv = transformFV(fv, cspace(end), [0 0]);
