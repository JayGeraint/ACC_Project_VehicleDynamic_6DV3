function comparison = compare_fig14
% Independent reference anchors. Source: Fei & Wang (2021), p. 4430, Fig. 14.
% Intermediate gap and speed readings are approximate to ~2 m and ~2 km/h.
root=fileparts(mfilename('fullpath'));
folder=fullfile(root,'results');
D=readtable(fullfile(folder,'fig14_deceleration.csv'));
t=[0;10;15;20;30];
paperGap=[75;55;42;38;38];
paperSpeed=[60;52;34;30;30];
gap=interp1(D.Time_s,D.Gap_m,t);
speed=interp1(D.Time_s,3.6*D.Ego_mps,t);
comparison=table(t,paperGap,gap,gap-paperGap,paperSpeed,speed,speed-paperSpeed, ...
    'VariableNames',{'Time_s','PaperGapApprox_m','ModelGap_m','GapDifference_m', ...
    'PaperEgoApprox_kmh','ModelEgo_kmh','SpeedDifference_kmh'});
writetable(comparison,fullfile(folder,'fig14_paper_comparison.csv'));
disp(comparison)
end
