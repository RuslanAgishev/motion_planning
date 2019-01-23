function out = transformFV (fv, theta, t)
% Rotate and translate a patch data structure

out.faces = fv.faces;

c = cosd(theta);
s = sind(theta);

out.vertices = bsxfun(@plus, fv.vertices*[c s; -s c], t);
