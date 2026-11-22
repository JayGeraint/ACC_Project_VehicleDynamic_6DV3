function errors = verify_carsim_interface
% Replay the MATLAB Fig. 14 trace through the native-unit interface and
% compare all controller outputs. This checks wiring and units only.
root=fileparts(mfilename('fullpath')); addpath(root);
mdl='acc_carsim_interface';
c=baseline_case(4);
D=readtable(fullfile(root,'results',c.name+".csv"));
if ~isfile(fullfile(root,[mdl '.slx'])), build_carsim_interface; end
load_system(fullfile(root,[mdl '.slx']));
origin=12.5;
t=D.Time_s;
vs=[D.Engine_rpm,D.GearRatio,3.6*D.Ego_mps,D.EgoFront_m+origin,D.Accel_mps2/9.80665];
ds=Simulink.SimulationData.Dataset;
ds=ds.addElement(c.scene,'Scene');
ds=ds.addElement(timeseries(vs,t),'VS');
in=Simulink.SimulationInput(mdl);
in=in.setVariable('initialXo_m',origin,'Workspace',mdl);
in=in.setExternalInput(ds);
out=sim(in);
expected={ [D.Throttle D.Pressure_MPa], [D.TargetValid D.Gap_m D.Relative_mps], ...
   [D.Reference_mps D.DesiredGap_m D.Mode D.StopHold], D.Demand_mps2 };
errors=zeros(4,1);
for k=1:4
    y=out.yout.getElement(k).Values;
    a=squeeze(y.Data);
    if size(a,1)~=numel(t), a=a'; end
    assert(isequal(size(a),size(expected{k})) && max(abs(y.Time-t))<1e-8);
    errors(k)=max(abs(a-expected{k}),[],'all');
end
T=table(["Pedal commands";"Range sensor";"Reference and mode";"Accel demand"],errors, ...
   'VariableNames',{'OutputGroup','ReplayMaxAbsoluteDifference'});
writetable(T,fullfile(root,'results','carsim_interface_replay.csv'));
disp(T); assert(all(errors<1e-7),'Native-unit interface replay differs from baseline.');
end
