function a = speed_control(ref,ego)
% Inner P speed feedback, no integral accumulation across mode transitions.
% Local gains and slew limits. Requests refer to net longitudinal acceleration.
persistent previous
if isempty(previous)
    previous = 0;
end
demand = min(max(2.4*(ref(1)-ego(2)),-4),2.8);
if ref(4)>0.5
    demand = -1.0;
end
a = previous + min(max(demand-previous,-6*0.02),6*0.02);
previous = a;
end
