function u = inverse_actuators(a,ego,ref)
% Drive/brake hysteresis around the zero-pedal coasting acceleration.
% Throttle is normalized, master-cylinder pressure is in MPa.
persistent driving
if isempty(driving)
    driving = true;
end
[loss,fullForce,mass] = baseline_forces(ego(2),ego(4),ego(5));
coast = -loss/mass;
if a>coast+0.05
    driving = true;
elseif a<coast-0.05
    driving = false;
end
force = mass*(a-coast);
throttle = 0;
pressure = 0;
if driving
    throttle = min(max(force/fullForce,0),1);
else
    torque = max(-force*0.304,0);
    % Two front (350 N m/MPa) and two rear (200 N m/MPa) brakes.
    % Above 2 MPa the rear pressure slope falls to 0.3.
    if torque <= 2200
        pressure = torque/1100;
    else
        pressure = 2+(torque-2200)/820;
    end
    pressure = min(pressure,8);
end
if ref(4)>0.5
    throttle = 0; pressure = max(pressure,0.6); driving = false;
end
u = [throttle;pressure];
end
