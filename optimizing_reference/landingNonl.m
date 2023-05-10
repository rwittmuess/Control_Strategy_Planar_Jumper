function [c,ceq] = landingNonl(k,tl_min,tl_end,lF,dlF)
    k31 = k(1);
    k32 = k(2);
    k33 = k(3);
    g = 9.81;
    
    eq1 = -k31*1/(cosh(k32*(0 - tl_min))^2)-k33 - lF;
    eq2 = k31*2*k32*sinh(k32*(0-tl_min))*cosh(k32*(0-tl_min))*k32 - dlF;

    ceq = [eq1; eq2];

    %%% inequality constraints c(k) <= 0 %%%
    % lref(t) >= 0.3
    % lref(t) <= 0.71
    % lref(td_LO) <= 0.6
    % lref(td_LO) >= 0.5
    
    c = [];
    j=0;
    for i = 0:0.01:tl_end
        j = j+1;
        c(j) = -(-k31*1/(cosh(k32*(i - tl_min))^2)-k33) + 0.3;
        j = j+1;
        c(j) = -k31*1/(cosh(k32*(i - tl_min))^2)-k33 - 0.71;
    end
    j = j+1;
    c(j) = 2*k31*k32*tanh(k32*(tl_end-tl_min))*sech(k32*(tl_end-tl_min)) - 0.05;

    
end