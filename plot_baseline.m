function plot_baseline(id,c,D,folder)
% All curves come from the Simulink run. Figure 14 markers are paper read-offs.
f=figure('Visible','off','Color','w','Position',[80 80 940 880]);
cleanup=onCleanup(@()close(f));
tl=tiledlayout(f,4,1,'TileSpacing','compact','Padding','compact');
titles = ["Fig. 11 | Sinusoidal cruise command","Fig. 12 | Multi-stage cruise command", ...
    "Fig. 13 | Continuous following","Fig. 14 | Lead-vehicle deceleration", ...
    "Course | Stop and restart","Course | Steady cruise"];
title(tl,titles(id)+"   /   MATLAB sedan",'FontWeight','bold');
t=D.Time_s;
ax=nexttile; hold on
plot(t,D.Reference_mps*3.6,'--','LineWidth',1.4,'DisplayName','Controller reference');
if any(D.TargetValid)
    plot(t,D.Lead_mps*3.6,':','LineWidth',1.8,'DisplayName','Lead');
end
plot(t,D.Ego_mps*3.6,'LineWidth',1.6,'DisplayName','Ego');
if id==4
    paper=readtable(fullfile(folder,'fig14_paper_comparison.csv'));
    plot(paper.Time_s,paper.PaperEgoApprox_kmh,'kd','MarkerFaceColor','w', ...
        'DisplayName','Paper ego (approx.)');
end
ylabel('Speed (km/h)'); grid on; legend('Location','best','NumColumns',2)
ax(2)=nexttile;
if any(D.TargetValid)
    plot(t,D.DesiredGap_m,'--','LineWidth',1.4,'DisplayName','CTH target'); hold on
    plot(t,D.Gap_m,'-','LineWidth',1.4,'DisplayName','Measured');
    ylabel('Bumper gap (m)');
    if id==4
        plot(paper.Time_s,paper.PaperGapApprox_m,'kd','MarkerFaceColor','w', ...
            'DisplayName','Paper gap (approx.)');
        xline(9,':','Paper switch','HandleVisibility','off');
        onset=find(D.Mode==1,1);
        if ~isempty(onset)
            xline(t(onset),'--','Model switch', ...
                'LabelVerticalAlignment','bottom','HandleVisibility','off');
        end
    end
    legend('Location','best');
else
    plot(t,(D.Ego_mps-D.Reference_mps)*3.6,'LineWidth',1.25);
    yline(0,':'); ylabel('Speed error (km/h)');
end
grid on
ax(3)=nexttile;
plot(t,D.Demand_mps2,'--',t,D.Accel_mps2,'-','LineWidth',1.25);
ylabel('Accel. (m/s^2)'); legend('Demand','Actual','Location','best'); grid on
ax(4)=nexttile;
yyaxis left; plot(t,D.Throttle,'LineWidth',1.35); ylabel('Throttle (0–1)'); ylim([0 1.05]);
yyaxis right; plot(t,D.Pressure_MPa,'LineWidth',1.35); ylabel('Brake (MPa)');
ylim([0 max(1,max(D.Pressure_MPa)*1.12)]); grid on
xlabel('Time (s)'); linkaxes(ax,'x'); xlim([0 c.duration]);
exportgraphics(f,fullfile(folder,c.name+".png"),'Resolution',150);
end
