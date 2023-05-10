function [c,ceq] = jumpOffNonl(k, td_LO,dld_LO,linit,dlinit)
    k11 = k(1);
    k12 = k(2);
    k13 = k(3);
    k14 = k(4);
    g = 9.81;

    eq1 = k11 * k12 * sech(0)^2 - g*td_LO + k13 - dld_LO;
    eq2 = k11 * k12 * sech(k12*(0-td_LO))^2 + k13 - dlinit;

    ceq = [eq1; eq2];
    
    %%% inequality constraints c(k) <= 0 %%%
    % lref(t) >= 0.3
    % lref(t) <= 0.71
    % lref(td_LO) <= 0.6
    % lref(td_LO) >= 0.5
    
    c = [];
    j=0;
    for i = 0:0.01:td_LO
        j = j+1;
        c(j) = -(k11 * tanh(k12 * (i - td_LO)) - 0.5*g*i^2 + k13*i + k14) + 0.3;
        j = j+1;
        c(j) = k11 * tanh(k12 * (i - td_LO)) - 0.5*g*i^2 + k13*i + k14 - 0.71;
    end
end
    