function ego = carsim_feedback(y,initialXo)
% Paper Fig. 6 export order. Confirm native units in the active VS dataset.
% AV_Eng rpm, Rgear_TR ratio, Vx km/h, Xo m, Ax g.
% Straight road, fixed heading. Local s=0 is the initial ego front bumper.
ego = [y(4)-initialXo; y(3)/3.6; y(5)*9.80665; y(1); y(2)];
end
