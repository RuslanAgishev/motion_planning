function [collFree] = isCollisionFree(Obs,xy)

% collFree=true;
% w = 0.5;
% Obs{1} = [0 0;5 0;5 w;0 w]';
% Obs{2} = [0 0;2*w 0;w 10;0 10]';
% Obs{3} = [0 10-w;5 10;5 10+w;0 10+w]';
% Obs{4} = [5-w 0;5+w 0;5+w 5;5 5]';
% Obs{5} = [5-w 10+w;5+w 10+w;5+w 7;5 7]';
% Obs{6} = [4 5;5+w 5;5+w 5+w;4 5+w]';
% Obs{7} = [4 7;5+w 7;5+w 7+w;4 7+w]';
% 
% xy = [2;10]; %as an examle

collFree=true;

for k = 1:length(Obs)
    O=Obs{k};
    x=O(1,:);
    y=O(2,:);
    k = convhull(x,y);
    [in,on] = inpolygon(xy(1),xy(2),x(k),y(k));
    if or(in,on) 
        collFree=false;
    end
end


end