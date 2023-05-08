function [c,ceq] = flightNonl(k,tz_Fmax, zd_Fmax)
    k21 = k(1);
    k22 = k(2);
    k23 = k(3);
    g = 9.81;

    % zFref = k21 * 1 / (cosh(k22 * (t - tz_Fmax))^2) - k23
    % zFref' = -2*k21*k22*sinh(k22*(t-tz_Fmax))*cosh(k22*(t-tz_Fmax))*k22 + k23
    
    %%% equality constraints c(k) = 0 %%%
    % zFref(0) = 0
    eq1 = k21 * 1 / (cosh(k22 * (0 - tz_Fmax))^2) - k23;
%     % zFref(2*tz_Fmax) = 0
%     eq2 = k21 * 1 / (cosh(k22 * (2*tz_Fmax - tz_Fmax))^2) - k23;
    % zFref'(0) = 0
%     eq3 = -2*k21*k22*sinh(k22*(0-tz_Fmax))*cosh(k22*(0-tz_Fmax))*k22 + k23;
    % zFref'(2*tz_Fmax) = 0
%     eq4 = -2*k21*k22*sinh(k22*(2*tz_Fmax-tz_Fmax))*cosh(k22*(2*tz_Fmax-tz_Fmax))*k22 + k23;
    % zFref(tz_Fmax) = zd_Fmax
    eq5 = k21 * 1 / (cosh(0)^2) - k23 - zd_Fmax;

    ceq = [eq1;eq5];
    
    %%% inequality constraints c(k) <= 0 %%%
    % lref(t) >= 0.3
    % lref(t) <= 0.71
    % lref(td_LO) <= 0.6
    % lref(td_LO) >= 0.5
    
    c = [];
    j=0;
%     for i = 0:0.01:2*tz_Fmax
% %         j = j+1;
% %         c(j) = - (k21 * 1 / (cosh(k22 * (i - tz_Fmax))^2) - k23);
% %         j = j+1;
% %         c(j) = k11 * tanh(k12 * (i - td_LO)) - 0.5*g*i^2 + k13*i + k14 - 0.71;
%     end
%     