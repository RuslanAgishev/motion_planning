function flag = triangle_intersection(P1, P2)
% triangle_test : returns true if the triangles overlap and false otherwise

%%% All of your code should be between the two lines of stars.
% *******************************************************************

% reference: https://habr.com/post/148325/
% Consider 1st triangle ABC. Let AB be the stright line and its vertice C
% is a point. We want to determine if C lies on the right or on the left
% side relative to line AB.
% After that we can check if all 3 vertices of the 2nd triangle MNK lie on
% the opposite site relative to the line AB. If so, there is no
% intersection between the 2 triangles.
% We should check this condition for all 3 possible configureations of a 
% strgiht line and a point: AB and point C, BC and point A, AC and point B.

A = P1(1,:); B = P1(2,:); C = P1(3,:); % 1st triangle
M = P2(1,:); N = P2(2,:); K = P2(3,:); % 2nd triangle

debug = 0;
if debug
    close all
    triangle1 = polyshape([A(1),B(1),C(1)],[A(2),B(2),C(2)]);
    triangle2 = polyshape([M(1),N(1),K(1)],[M(2),N(2),K(2)]);
    figure()
    plot(triangle1)
    hold on
    plot(triangle2)
end

%% check triangle ABC
% left and right
if strcmp( linepoint(A,B, C), 'left' ) && strcmp( linepoint(A,B, M), 'right' )...
                                       && strcmp( linepoint(A,B, N), 'right' )...
                                       && strcmp( linepoint(A,B, K), 'right' )
    flag = false; % do not overlap
    if debug, fprintf('\n no overlap'); end
    return
end
if strcmp( linepoint(A,C, B), 'left' ) && strcmp( linepoint(A,C, M), 'right' )...
                                       && strcmp( linepoint(A,C, N), 'right' )...
                                       && strcmp( linepoint(A,C, K), 'right' )
    flag = false; % do not overlap
    if debug, fprintf('\n no overlap'); end
    return
end
if strcmp( linepoint(B,C, A), 'left' ) && strcmp( linepoint(B,C, M), 'right' )...
                                       && strcmp( linepoint(B,C, N), 'right' )...
                                       && strcmp( linepoint(B,C, K), 'right' )
    flag = false; % do not overlap
    if debug, fprintf('\n no overlap'); end
    return
end

% right and left
if strcmp( linepoint(A,B, C), 'right' ) && strcmp( linepoint(A,B, M), 'left' )...
                                        && strcmp( linepoint(A,B, N), 'left' )...
                                        && strcmp( linepoint(A,B, K), 'left' )
    flag = false; % do not overlap
    if debug, fprintf('\n no overlap'); end
    return
end
if strcmp( linepoint(A,C, B), 'right' ) && strcmp( linepoint(A,C, M), 'left' )...
                                        && strcmp( linepoint(A,C, N), 'left' )...
                                        && strcmp( linepoint(A,C, K), 'left' )
    flag = false; % do not overlap
    if debug, fprintf('\n no overlap'); end
    return
end
if strcmp( linepoint(B,C, A), 'right' ) && strcmp( linepoint(B,C, M), 'left' )...
                                        && strcmp( linepoint(B,C, N), 'left' )...
                                        && strcmp( linepoint(B,C, K), 'left' )
    flag = false; % do not overlap
    if debug, fprintf('\n no overlap'); end
    return
end

%% check triangle MNK
% left and right
if strcmp( linepoint(M,N, K), 'left' ) && strcmp( linepoint(M,N, A), 'right' )...
                                       && strcmp( linepoint(M,N, B), 'right' )...
                                       && strcmp( linepoint(M,N, C), 'right' )
    flag = false; % do not overlap
    if debug, fprintf('\n no overlap'); end
    return
end
if strcmp( linepoint(M,K, N), 'left' ) && strcmp( linepoint(M,K, A), 'right' )...
                                       && strcmp( linepoint(M,K, B), 'right' )...
                                       && strcmp( linepoint(M,K, C), 'right' )
    flag = false; % do not overlap
    if debug, fprintf('\n no overlap'); end
    return
end
if strcmp( linepoint(N,K, M), 'left' ) && strcmp( linepoint(N,K, A), 'right' )...
                                       && strcmp( linepoint(N,K, B), 'right' )...
                                       && strcmp( linepoint(N,K, C), 'right' )
    flag = false; % do not overlap
    if debug, fprintf('\n no overlap'); end
    return
end

% right and left
if strcmp( linepoint(M,N, K), 'right' ) && strcmp( linepoint(M,N, A), 'left' )...
                                        && strcmp( linepoint(M,N, B), 'left' )...
                                        && strcmp( linepoint(M,N, C), 'left' )
    flag = false; % do not overlap
    if debug, fprintf('\n no overlap'); end
    return
end
if strcmp( linepoint(M,K, N), 'right' ) && strcmp( linepoint(M,K, A), 'left' )...
                                        && strcmp( linepoint(M,K, B), 'left' )...
                                        && strcmp( linepoint(M,K, C), 'left' )
    flag = false; % do not overlap
    if debug, fprintf('\n no overlap'); end
    return
end
if strcmp( linepoint(N,K, M), 'right' ) && strcmp( linepoint(N,K, A), 'left' )...
                                        && strcmp( linepoint(N,K, B), 'left' )...
                                        && strcmp( linepoint(N,K, C), 'left' )
    flag = false; % do not overlap
    if debug, fprintf('\n no overlap'); end
    return
end

flag = true;
if debug, fprintf('\n no overlap'); end
% *******************************************************************

end

