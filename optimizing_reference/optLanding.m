function [k31, k32, k33] = optLanding(tl_min,tl_end,lF,dlF)
    k0 = [0.1;2;0.58];
    
    Aineq = []; Bineq = []; % we have no linear inequality constraints
    Aeq = []; Beq = []; % we have no equality constraints
    
    LB = [-Inf, -Inf, -Inf];
    UB = [Inf, Inf, Inf];
    
    options = optimset('display','iter','diffmaxchange',1.1*1e-5,'diffminchange',1e-5,'MaxFunEvals',200000,'MaxIter',200000,'TolCon',0.05,'Display','notify');
    
    [k,~] = ...
        fmincon(@landingCost,k0,Aineq,Bineq,Aeq,Beq,LB,UB,@landingNonl,options,tl_min,tl_end,lF,dlF);
    
    k31 = k(1);
    k32 = k(2);
    k33 = k(3);
end