function metrics = run_baseline(ids)
% Run all six cases, or e.g. run_baseline(4). Requires MATLAB and Simulink.
% Full-run summary files are replaced only when all six cases are requested.
if nargin==0, ids = 1:6; end
validateattributes(ids,{'numeric'},{'vector','integer','>=',1,'<=',6,'nonempty'});
assert(numel(ids)==numel(unique(ids)),'Use each scenario ID once.');
root = fileparts(mfilename('fullpath'));
addpath(root);
mdl = 'acc_paper_baseline';
if ~isfile(fullfile(root,[mdl '.slx']))
    build_baseline;
end
results = fullfile(root,'results');
if ~isfolder(results), mkdir(results); end
load_system(fullfile(root,[mdl '.slx']));
rows = cell(numel(ids),1);
checks = cell(numel(ids),1);
for k=1:numel(ids)
    id = ids(k);
    c = baseline_case(id);
    in = Simulink.SimulationInput(mdl);
    in = in.setVariable('sceneInput',c.scene,'Workspace',mdl);
    in = in.setVariable('ego0',c.ego0,'Workspace',mdl);
    in = in.setVariable('power0',c.power0,'Workspace',mdl);
    in = in.setModelParameter('StopTime',num2str(c.duration));
    out = sim(in);
    D = decode(out.accLog);
    [rows{k},checks{k}] = evaluate_case(id,c,D);
    writetable(D,fullfile(results,c.name+".csv"));
    writetable(checks{k},fullfile(results,c.name+"_checks.csv"));
    if id==4
        compare_fig14;
    end
    plot_baseline(id,c,D,results);
    fprintf('%s   %d/%d checks passed\n',c.name,sum(checks{k}.Pass),height(checks{k}));
end
metrics = vertcat(rows{:});
allChecks = vertcat(checks{:});
if isequal(sort(ids(:))',(1:6))
    writetable(metrics,fullfile(results,'metrics.csv'));
    writetable(allChecks,fullfile(results,'checks.csv'));
end
disp(metrics);
if any(~allChecks.Pass)
    disp(allChecks(~allChecks.Pass,:));
    error('ACC:Acceptance','One or more engineering checks failed. See the case results.');
end
end

function D = decode(log)
A = squeeze(log.Data);
if size(A,1)~=numel(log.Time), A=A'; end
names = {'Time_s','Cruise_mps','Lead_mps','LeadRear_m','LeadExists','EgoFront_m', ...
    'Ego_mps','Accel_mps2','Engine_rpm','GearRatio','TargetValid','Gap_m', ...
    'Relative_mps','Reference_mps','DesiredGap_m','Mode','StopHold', ...
    'Demand_mps2','Throttle','Pressure_MPa'};
assert(size(A,2)==numel(names)-1,'Unexpected log dimensions.');
D = array2table([log.Time(:),A],'VariableNames',names);
noTarget = ~logical(D.TargetValid);
D.Gap_m(noTarget)=NaN;
D.Relative_mps(noTarget)=NaN;
D.DesiredGap_m(noTarget)=NaN;
D.Lead_mps(~logical(D.LeadExists))=NaN;
D.LeadRear_m(~logical(D.LeadExists))=NaN;
end

function [M,C] = evaluate_case(id,c,D)
t=D.Time_s;
v=D.Ego_mps*3.6;
e=(D.Ego_mps-D.Reference_mps)*3.6;
valid = logical(D.TargetValid);
gapErr = D.Gap_m-D.DesiredGap_m;
accel = D.Accel_mps2;
rows = cell(0,2);
sceneData = c.scene.Data;
sampleError = abs(D.Cruise_mps-sceneData(:,1));
if any(valid)
    sampleError = [sampleError; abs(D.Lead_mps(valid)-sceneData(valid,2)); ...
        abs(D.LeadRear_m(valid)-sceneData(valid,3))];
end
test("Scenario playback matches prescribed samples",all(sampleError<1e-8) && ...
    isequal(D.LeadExists,sceneData(:,4)));
test("Time grid, finite controls and ego state",height(D)==round(c.duration/c.Ts)+1 && ...
    max(abs(diff(t)-c.Ts))<1e-8 && all(isfinite([v;e;accel;D.EgoFront_m; ...
    D.Engine_rpm;D.GearRatio;D.Demand_mps2;D.Throttle;D.Pressure_MPa])));
test("Commands in bounds, no simultaneous pedals", ...
    all(D.Throttle>=0 & D.Throttle<=1 & D.Pressure_MPa>=0 & D.Pressure_MPa<=8) ...
    && ~any(D.Throttle>1e-9 & D.Pressure_MPa>1e-9));
test("Nonnegative speed, monotone travel",all(v>=0) && all(diff(D.EgoFront_m)>=-1e-9));
test("Acceleration bounded within [-4.5,3.5] m/s^2",min(accel)>=-4.5 && max(accel)<=3.5);
if any(valid)
    test("Bumper clearance positive for full case",all(valid) && all(D.Gap_m>0));
    test("Sensor gap and range rate match bumper geometry", ...
        max(abs(D.Gap_m-(D.LeadRear_m-D.EgoFront_m)))<1e-8 && ...
        max(abs(D.Relative_mps-(D.Lead_mps-D.Ego_mps)))<1e-8);
else
    test("Free road stays in cruise mode",all(D.Mode==2));
end
firstFollow = firstTime(t,D.Mode==1);
settled30 = NaN;
stopStart = NaN;
restart = NaN;
switches = sum(diff(D.Mode)~=0);
switch id
    case 1
        test("Sine cruise RMSE after first 5 s below 4 km/h", ...
            sqrt(mean(e(t>=5).^2))<4);
    case 2
        test("Final 10 s within 1 km/h of cruise setting",max(abs(e(t>=40)))<1);
        test("Both acceleration and braking exercised",max(D.Throttle)>0.1 && max(D.Pressure_MPa)>0.1);
    case 3
        test("Continuous following, gap RMSE below 5 m", ...
            all(D.Mode==1) && sqrt(mean(gapErr.^2))<5);
    case 4
        test("One cruise-to-follow transition during lead ramp", ...
            switches==1 && D.Mode(1)==2 && firstFollow>=5 && firstFollow<=10);
        settled30 = settledTime(t,abs(v-30)<=1,10);
        test("Settles within 1 km/h of 30, final gap error below 3 m", ...
            isfinite(settled30) && abs(gapErr(end))<3);
    case 5
        stopStart=firstTime(t,t>=18 & t<53 & v<1e-6);
        restart=firstTime(t,t>=53 & v>1);
        test("Full stop and brake hold for final 8 s of lead stop", ...
            all(v(t>=45 & t<53)<1e-6) && all(D.StopHold(t>=45 & t<53)==1));
        test("Stopped clearance within 1 m of standstill target",abs(gapErr(t==50))<1);
        test("Restarts, reaches within 1 km/h of 45 by end", ...
            isfinite(restart) && restart<63 && all(abs(v(t>=90)-45)<1));
    case 6
        test("Settled within 0.5 km/h throughout final 10 s",all(abs(v(t>=30)-60)<0.5));
end
C = table(repmat(c.name,size(rows,1),1),string(rows(:,1)),cell2mat(rows(:,2)), ...
          'VariableNames',{'Case','EngineeringCheck','Pass'});
gapMin = NaN; gapRMSE = NaN;
if any(valid)
    gapMin = min(D.Gap_m(valid)); gapRMSE = sqrt(mean(gapErr(valid).^2));
end
M = table(c.name,sqrt(mean(e.^2)),gapMin,gapRMSE,min(accel),max(accel), ...
    sqrt(mean((diff(accel)./diff(t)).^2)),switches,firstFollow,settled30,stopStart, ...
    restart,v(end),D.Gap_m(end),all(C.Pass),'VariableNames', ...
    {'Case','ReferenceRMSE_kmh','MinGap_m','GapRMSE_m','MinAccel_mps2','MaxAccel_mps2', ...
     'JerkRMS_mps3','ModeChanges','FirstFollow_s','Settled30_s','AtRest_s', ...
     'RestartAbove1kmh_s','FinalSpeed_kmh','FinalGap_m','Passed'});
    function test(name,value)
        rows(end+1,:)={char(name),logical(value)};
    end
end

function value = firstTime(t,mask)
i=find(mask,1); value=NaN;
if ~isempty(i), value=t(i); end
end

function value = settledTime(t,ok,after)
i=find(t>=after);
bad=find(~ok(i),1,'last');
value=NaN;
if isempty(bad)
    value=t(i(1));
elseif bad<numel(i)
    value=t(i(bad+1));
end
end
