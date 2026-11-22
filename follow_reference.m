function ref = follow_reference(scene,ego,z)
% Output [speed request m/s, desired gap m, mode (1 ACC / 2 CC), hold].
% Filtered derivative of d - d0 - h*v, directly from the sensor and ego ax.
persistent following integral deriv hold
if isempty(following)
    following = false; integral = 0; deriv = 0; hold = false;
end
Ts = 0.02;
h = 3.6;
d0 = 5;
v = ego(2);
desiredGap = d0+h*v;
error = z(2)-desiredGap;
if ~logical(z(1))
    following = false;
elseif following
    if error>10 && z(3)>=0
        following = false;
    end
elseif error<=0
    following = true;
end
request = scene(1);
if z(1)>0.5
    deriv = deriv + Ts/(0.18+Ts)*(z(3)-h*ego(3)-deriv);
else
    deriv = 0;
end
if following
    correction = 0.8*error + integral + 0.3*deriv; % km/h
    dv = min(max(correction,-35),35);
    request = min(max(v+dv/3.6,0),scene(1));
    applied = (request-v)*3.6;
    integral = integral + Ts*(0.07*error + 0.035*(applied-correction));
    % Course stop-and-go latch. Release when the lead begins pulling away.
    leadSpeed = v+z(3);
    if hold
        hold = leadSpeed<0.5 && z(2)<d0+2;
    else
        hold = leadSpeed<0.10 && v<0.55 && z(2)<d0+0.8;
    end
    if hold
        request = 0; integral = 0; deriv = 0;
    end
else
    integral = 0; hold = false;
end
ref = [request;desiredGap;2-double(following);double(hold)];
end
