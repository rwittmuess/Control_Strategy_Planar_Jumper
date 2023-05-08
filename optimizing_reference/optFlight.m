function [k21, k22, k23] = optFlight(tz_Fmax, zd_Fmax)
    k0 = [0.13;25;0.01];
    
    Aineq = []; Bineq = []; % we have no linear inequality constraints
    Aeq = []; Beq = []; % we have no equality constraints
    
    LB = [-Inf, -Inf, -Inf];
    UB = [Inf, Inf, Inf];
    
    options = optimset('display','iter','diffmaxchange',1.1*1e-5,'diffminchange',1e-5,'MaxFunEvals',200000,'MaxIter',200000,'TolCon',0.05,'Display','notify');
    
    [k,~] = ...
        fmincon(@flightCost,k0,Aineq,Bineq,Aeq,Beq,LB,UB,@flightNonl,options,tz_Fmax, zd_Fmax);
    
    k21 = k(1);
    k22 = k(2);
    k23 = k(3);
end