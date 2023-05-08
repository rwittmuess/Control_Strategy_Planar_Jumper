function [c,ceq] = reJumpNonl(k,treJump, ld_LO, dld_LO, lF, dlF)
    k41 = k(1);
    k42 = k(2);
    k43 = k(3);
    k44 = k(4);
    g = 9.81;

    % lref = k41 * tanh(k42 * (t - t_rejump)) - 0.5*g*t^2 + k43*t + k44
    % lref' = k41 * k42 * sech(k42*(t-t_rejump))^2 - g*t + k43
    
    %%% equality constraints c(k) = 0 %%%
    % lref(0) = lF
    eq1 = k41 * tanh(k42 * (0 - treJump)) + k44 - lF;
    % lref'(0) = dlF
    eq2 = k41 * k42 * sech(k42*(0-treJump))^2 + k43 - dlF;
    % lref(t_rejump) = ld_L0
% %     eq3 = k41 * tanh(0) - 0.5*g*treJump^2 + k43*treJump + k44 - ld_LO;
% %     % lref'(t_rejump) = dld_L0
% %     eq4 = k41 * k42 * sech(0)^2 - g*treJump + k43 - dld_LO;

    ceq = [eq1; eq2];
    
    %%% inequality constraints c(k) <= 0 %%%
    % lref(t) >= 0.3
    % lref(t) <= 0.71
    % lref(td_LO) <= 0.6
    % lref(td_LO) >= 0.5
    
    c = [];
    j=0;
    for i = 0:0.01:treJump
        j = j+1;
        c(j) = -(k41 * tanh(k42 * (i - treJump)) - 0.5*g*i^2 + k43*i + k44) + 0.3;
        j = j+1;
        c(j) = k41 * tanh(k42 * (i - treJump)) - 0.5*g*i^2 + k43*i + k44 - 0.71;
    end
%     j = j+1;
%     c(j) = -treJump;
end
    