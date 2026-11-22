function c = baseline_case(id)
% Profiles use km/h here. The model interface uses SI units.
names = ["fig11_sine","fig12_multistage","fig13_follow","fig14_deceleration", ...
         "course_stop_go","course_steady"];
durations = [50 50 40 30 95 40];
initial = [40 40 60 60 45 40];
gaps = [0 0 65 75 50 0];
assert(isscalar(id) && ismember(id,1:numel(names)),'Choose a case from 1 to 6.')
c.name = names(id);
c.duration = durations(id);
c.v0 = initial(id)/3.6;
c.Ts = 0.02;
t = (0:c.Ts:c.duration)';
hasTarget = ismember(id,[3 4 5]);
lead = zeros(size(t));
switch id
    case 1  % Fig. 11, period from the text, initial speed estimated from plot
        setSpeed = 50 + 10*sin(2*pi*t/10);
    case 2  % Piecewise-linear breakpoints estimated from Fig. 12
        setSpeed = interp1([0 4 7 14 18 21 27 31 35 50], ...
                           [50 50 38 38 60 60 29 29 50 50],t);
    case 3  % Text gives 60--70 km/h. Fig. 13 suggests an ~8 s period.
        lead = 65 + 5*sin(2*pi*t/8-asin(1));
        setSpeed = 80*ones(size(t));
    case 4  % Fig. 14, 5--10 s lead deceleration ramp
        lead = interp1([0 5 10 30],[60 60 30 30],t);
        setSpeed = 60*ones(size(t));
    case 5  % Course stop, queue at rest, then restart
        lead = interp1([0 8 18 53 63 95],[45 45 0 0 45 45],t);
        setSpeed = 55*ones(size(t));
    case 6
        setSpeed = 60*ones(size(t));
end
% Ego front bumper starts at s=0. Lead coordinate is its rear bumper.
rearPosition = gaps(id) + cumtrapz(t,lead/3.6);
c.scene = timeseries([setSpeed/3.6,lead/3.6,rearPosition, ...
                     repmat(double(hasTarget),size(t))],t);
[rpm,ratio,gear] = baseline_gear(c.v0,0);
c.ego0 = [0;c.v0;0;rpm;ratio];
% Equilibrium throttle prevents a spurious open-pedal transient at t=0.
[roadLoad,fullForce] = baseline_forces(c.v0,rpm,ratio);
c.power0 = [roadLoad/fullForce;0;gear];
end
