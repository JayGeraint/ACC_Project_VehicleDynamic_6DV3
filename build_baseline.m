function build_baseline
% Rebuild the Simulink controller library and default Fig. 14 test bench.
root = fileparts(mfilename('fullpath'));
addpath(root);
lib = 'acc_control_lib';
mdl = 'acc_paper_baseline';
for name = {mdl,lib}
    if bdIsLoaded(name{1})
        assert(strcmp(get_param(name{1},'Dirty'),'off'), ...
            'Save your open model before rebuilding it.')
        close_system(name{1},0);
    end
end
load_system('simulink');
new_system(lib,'Library');
set_param(lib,'Lock','off');
controller = [lib '/ACC controller'];
add_block('built-in/Subsystem',controller,'Position',[100 100 300 200]);
port(controller,'Inport','Scene',1,[25 80 55 94]);
port(controller,'Inport','Ego',2,[25 200 55 214]);
chart(controller,'Range sensor','range_sensor',[115 70 235 125],root);
chart(controller,'Distance and mode','follow_reference',[305 60 455 145],root);
chart(controller,'Speed loop','speed_control',[525 90 640 155],root);
chart(controller,'Inverse dynamics','inverse_actuators',[690 155 825 235],root);
port(controller,'Outport','Pedals',1,[905 176 935 190]);
port(controller,'Outport','Sensor',2,[900 35 930 49]);
port(controller,'Outport','Reference',3,[900 75 930 89]);
port(controller,'Outport','Accel demand',4,[900 115 930 129]);
wire(controller,{
    'Scene/1','Range sensor/1'; 'Ego/1','Range sensor/2'
    'Scene/1','Distance and mode/1'; 'Ego/1','Distance and mode/2'
    'Range sensor/1','Distance and mode/3'; 'Range sensor/1','Sensor/1'
    'Distance and mode/1','Speed loop/1'; 'Ego/1','Speed loop/2'
    'Speed loop/1','Inverse dynamics/1'; 'Ego/1','Inverse dynamics/2'
    'Distance and mode/1','Inverse dynamics/3'; 'Inverse dynamics/1','Pedals/1'
    'Distance and mode/1','Reference/1'; 'Speed loop/1','Accel demand/1'});
set_param(lib,'Lock','on');
save_system(lib,fullfile(root,[lib '.slx']));

new_system(mdl);
set_param(mdl,'SolverType','Fixed-step','Solver','FixedStepDiscrete', ...
    'FixedStep','0.02','StopTime','30','ReturnWorkspaceOutputs','on');
c = baseline_case(4);
mw = get_param(mdl,'ModelWorkspace');
assignin(mw,'sceneInput',c.scene);
assignin(mw,'ego0',c.ego0);
assignin(mw,'power0',c.power0);
add_block('simulink/Sources/From Workspace',[mdl '/Lead and cruise scenario'], ...
    'VariableName','sceneInput','Interpolate','on','SampleTime','0.02', ...
    'OutputAfterFinalValue','Holding final value', ...
    'Position',[25 70 185 110]);
add_block(controller,[mdl '/ACC controller'],'Position',[265 80 435 190]);
chart(mdl,'Sedan dynamics','sedan_step',[525 200 675 270],root);
add_block('simulink/Discrete/Unit Delay',[mdl '/Ego feedback'], ...
    'SampleTime','0.02','InitialCondition','ego0','Position',[720 210 800 245]);
add_block('simulink/Discrete/Unit Delay',[mdl '/Actuator and gear state'], ...
    'SampleTime','0.02','InitialCondition','power0','Position',[715 300 805 335]);
add_block('simulink/Signal Routing/Mux',[mdl '/Log channels'], ...
    'Inputs','6','Position',[865 45 870 165]);
add_block('simulink/Sinks/To Workspace',[mdl '/Record'], ...
    'VariableName','accLog','SaveFormat','Timeseries','Position',[920 88 1015 122]);
wire(mdl,{
    'Lead and cruise scenario/1','ACC controller/1'; 'Ego feedback/1','ACC controller/2'
    'ACC controller/1','Sedan dynamics/1'; 'Ego feedback/1','Sedan dynamics/2'
    'Actuator and gear state/1','Sedan dynamics/3'; 'Sedan dynamics/1','Ego feedback/1'
    'Sedan dynamics/2','Actuator and gear state/1'
    'Lead and cruise scenario/1','Log channels/1'; 'Ego feedback/1','Log channels/2'
    'ACC controller/2','Log channels/3'; 'ACC controller/3','Log channels/4'
    'ACC controller/4','Log channels/5'; 'ACC controller/1','Log channels/6'
    'Log channels/1','Record/1'});
set_param(mdl,'SimulationCommand','update');
save_system(mdl,fullfile(root,[mdl '.slx']));
close_system(mdl,0);
close_system(lib,0);
end

function chart(parent,name,file,position,root)
path = [parent '/' name];
add_block('simulink/User-Defined Functions/MATLAB Function',path,'Position',position);
rt = sfroot;
obj = rt.find('-isa','Stateflow.EMChart','Path',path);
obj.Script = fileread(fullfile(root,[file '.m']));
end

function port(parent,type,name,n,position)
add_block(['built-in/' type],[parent '/' name],'Port',num2str(n),'Position',position);
end

function wire(parent,pairs)
for i=1:size(pairs,1)
    add_line(parent,pairs{i,1},pairs{i,2},'autorouting','on');
end
end
