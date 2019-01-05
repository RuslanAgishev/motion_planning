function fv = boxFV (x1, x2, y1, y2)
% Make a simple box
fv.vertices = [x1 y1; x2 y1; x2 y2; x1 y2];
fv.faces = [1 2 3; 1 3 4];