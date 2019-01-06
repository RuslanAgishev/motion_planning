function fv = TwoLinkRobot (cspace_coords)
% TwoLinkRobot computes the vertices and face of a two link robot

% Parameters that define robot dimensions
w = 0.5;
l = 5;
w2 = 0.75;
l2 = 1;

link1.vertices = [0 -w; l -w; l w; 0 w];
link1.faces = [1 2 3; 1 3 4];

endeffector.vertices = [0 -w2; l2 -w2; 0 w2; l2 w2];
endeffector.faces = [1 2 3; 1 3 4];

link2 = appendFV(link1, transformFV(endeffector, 0, [l 0]));

% Transform link2

link2 = transformFV(link2, cspace_coords(1), [l 0]);

fv = appendFV(link1, link2);

fv = transformFV(fv, cspace_coords(2), [0 0]);