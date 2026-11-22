function [resistance,fullForce,effectiveMass] = baseline_forces(v,rpm,ratio)
% Level-road surrogate. Replace the load and torque curves with the CarSim
% dataset when calibrating the inverse model for a specific vehicle.
mass = 1420;                % assumed total running mass, kg
effectiveMass = mass*1.06;  % local effective inertia approximation
resistance = (mass*9.81*0.012+70)*min(max(v,0)/0.5,1) ...
             + 0.5*1.225*0.68*v^2;
rpmAxis = [750 1500 2500 3500 4500 5500 6000 6500];
fullTorque = [135 205 250 275 270 250 238 205]; % N m, approximate 150 kW class
T = interp1(rpmAxis,fullTorque,min(max(rpm,750),6500));
fullForce = T*max(ratio,0.1)*4.1*0.92/0.304;
end
