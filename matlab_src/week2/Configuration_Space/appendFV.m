function fv3 = appendFV (fv1, fv2)
% append two patch structures

n1 = size(fv1.vertices,1);

fv3.vertices = [fv1.vertices; fv2.vertices];
fv3.faces = [fv1.faces; fv2.faces + n1];
