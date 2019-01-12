function side = linepoint(A,B, C)
% figure()
% line([A(1),B(1)], [A(2),B(2)])
% hold on
% plot(A(1),A(2), '*')
% plot(B(1),B(2), 'o')
% plot(C(1),C(2), 'ro')

if cross( [B-A,0], [C-A,0] )>=0, side = 'left';
else, side = 'right';
end
end