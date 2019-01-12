% function [nbs, map] = neighbours(map, node)
function nbs = neighbours(map, node)
N = size(map,1);
nbs = [];
if node > 1 && map(node-1) ~= 2
%     map(node-1) = 4;
    nbs = [nbs, node-1];
end
if node < N^2 && map(node+1) ~= 2
%     map(node+1) = 4;
    nbs = [nbs, node+1];
end
if node > N && map(node-N) ~= 2
%     map(node-N) = 4;
    nbs = [nbs, node-N];
end
if node < N*(N-1) && map(node+N) ~= 2
%     map(node+N) = 4;
    nbs = [nbs, node+N];
end

end
