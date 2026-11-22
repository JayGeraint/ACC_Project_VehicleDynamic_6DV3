function [nextEgo,nextPower] = sedan_step(u,ego,power)
% Forward vehicle model. Both state vectors are delayed one tick in Simulink.
Ts = 0.02;
throttle = power(1)+(1-exp(-Ts/0.25))*(u(1)-power(1));
pressure = power(2)+(1-exp(-Ts/0.08))*(u(2)-power(2));
rearPressure = min(pressure,2) + 0.30*max(pressure-2,0);
brakeForce = 2*(350*pressure+200*rearPressure)/0.304;
[loss,traction,mass] = baseline_forces(ego(2),ego(4),ego(5));
net = (throttle*traction-brakeForce-loss)/mass;
v = max(ego(2)+Ts*net,0);
a = (v-ego(2))/Ts;
s = ego(1) + Ts*(v+ego(2))/2;
[rpm,ratio,gear] = baseline_gear(v,power(3));
nextEgo = [s;v;a;rpm;ratio];
nextPower = [throttle;pressure;gear];
end
