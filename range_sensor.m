function z = range_sensor(scene,ego)
% scene = [set speed, lead speed, lead rear s, target exists].
% ego = [front bumper s, speed, acceleration, engine rpm, transmission ratio].
% Negative gap remains visible for collision detection in a single-lane run.
gap = scene(3)-ego(1);
valid = scene(4)>0.5 && gap<=150;
z = [double(valid);0;0];
if valid
    z(2:3) = [gap;scene(2)-ego(2)];
end
end
