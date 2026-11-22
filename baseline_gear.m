function [rpm,ratio,gear] = baseline_gear(v,oldGear)
% Paper Fig. 4 ratios. Local speed-based shift schedule with hysteresis.
ratios = [3.538 2.060 1.404 1.000 0.713 0.582];
up = [15 32 51 69 90];
down = [10 26 46 62 83];
kph = max(0,v)*3.6;
if oldGear < 1
    gear = 1 + sum(kph >= up);
else
    gear = round(oldGear);
    if gear < 6 && kph >= up(gear)
        gear = gear+1;
    elseif gear > 1 && kph < down(gear-1)
        gear = gear-1;
    end
end
ratio = ratios(gear);
rpm = max(750,v/0.304*ratio*4.1*60/(2*pi));
end
