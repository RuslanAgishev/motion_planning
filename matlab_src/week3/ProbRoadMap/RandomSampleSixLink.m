function x = RandomSampleSixLink (obstacle)
% Generate a random freespace configuration for the robot

while true
    x = 360*rand(6,1);
    
    fv = SixLinkRobot (x);
    
    if (~CollisionCheck(fv, obstacle))
        return
    end
end