function out = my_linspace (P1, P2, n)
% Linearly interpolate between 2 points

a = linspace(0,1,n);

% Note use of outer product, columns are the interpolants
out = P2(:)*a + P1(:)*(1-a);