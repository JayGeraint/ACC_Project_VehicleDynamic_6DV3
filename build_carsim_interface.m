function build_carsim_interface
% Simulink-side interface for a VS S-function. No CarSim license needed to build.
root = fileparts(mfilename('fullpath')); addpath(root);
mdl='acc_carsim_interface';
if bdIsLoaded(mdl)
    assert(strcmp(get_param(mdl,'Dirty'),'off'),'Save your interface model before rebuilding.');
    close_system(mdl,0);
end
load_system(fullfile(root,'acc_control_lib.slx'));
new_system(mdl);
set_param(mdl,'SolverType','Fixed-step','Solver','FixedStepDiscrete', ...
    'FixedStep','0.02','StopTime','30','ReturnWorkspaceOutputs','on', ...
    'SaveOutput','on','OutputSaveName','yout','SaveFormat','Dataset');
mw=get_param(mdl,'ModelWorkspace'); assignin(mw,'initialXo_m',0);
add_block('built-in/Inport',[mdl '/Scene SI'],'Port','1','PortDimensions','4', ...
    'SampleTime','0.02','Position',[35 65 65 79]);
add_block('built-in/Inport',[mdl '/VS exports'],'Port','2','PortDimensions','5', ...
    'SampleTime','0.02','Position',[35 144 65 158]);
add_block('simulink/Sources/Constant',[mdl '/Initial Xo in m'],'Value','initialXo_m', ...
    'Position',[30 190 95 215]);
add_block('simulink/User-Defined Functions/MATLAB Function',[mdl '/Convert units'], ...
    'Position',[155 128 275 205]);
rt=sfroot; ch=rt.find('-isa','Stateflow.EMChart','Path',[mdl '/Convert units']);
ch.Script=fileread(fullfile(root,'carsim_feedback.m'));
add_block('acc_control_lib/ACC controller',[mdl '/ACC controller'], ...
    'Position',[355 70 515 200]);
for k=1:4
    labels={'VS imports','Sensor','Reference','Accel demand'};
    add_block('built-in/Outport',[mdl '/' labels{k}],'Port',num2str(k), ...
        'Position',[605 40*k+40 635 40*k+54]);
    add_line(mdl,['ACC controller/' num2str(k)],[labels{k} '/1'],'autorouting','on');
end
add_line(mdl,'Scene SI/1','ACC controller/1','autorouting','on');
add_line(mdl,'VS exports/1','Convert units/1','autorouting','on');
add_line(mdl,'Initial Xo in m/1','Convert units/2','autorouting','on');
add_line(mdl,'Convert units/1','ACC controller/2','autorouting','on');
set_param(mdl,'SimulationCommand','update');
save_system(mdl,fullfile(root,[mdl '.slx']));
close_system(mdl,0);
end
