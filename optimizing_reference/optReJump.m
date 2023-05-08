function [k41, k42, k43, k44] = optReJump(treJump,ld_LO, dld_LO, lF, dlF)
    k0 = [6;2;-2;6];
    
    Aineq = []; Bineq = []; % we have no linear inequality constraints
    Aeq = []; Beq = []; % we have no equality constraints
    
    LB = [-Inf, -Inf, -Inf, -Inf];
    UB = [Inf, Inf, Inf, Inf];
    
    options = optimset('display','iter','diffmaxchange',1.1*1e-5,'diffminchange',1e-5,'MaxFunEvals',200000,'MaxIter',200000,'TolCon',0.05,'Display','notify');
    
    [k,~] = ...
        fmincon(@reJumpCost,k0,Aineq,Bineq,Aeq,Beq,LB,UB,@reJumpNonl,options,treJump,ld_LO, dld_LO, lF, dlF);
    
    k41 = k(1);
    k42 = k(2);
    k43 = k(3);
    k44 = k(4);
end