function out = CollisionCheck (fv1, fv2)
% Determine if two sets of triangular faces overlap

n1 = size (fv1.faces, 1);
n2 = size (fv2.faces, 1);

for i = 1:n1
    P1 = fv1.vertices(fv1.faces(i,:), :);
    for j = 1:n2
        P2 = fv2.vertices(fv2.faces(j,:), :);
        
        if (triangle_intersection(P1,P2))
            out = true;
            return;
        end
    end
end

out = false;
