function [k11, k12, k13, k14] = optJumpOff(td_LO, dld_LO)
    k0 = [6;2;-2;6];

    linit = 0.5; % must be between 0.3 and 0.71
    dlinit = 0; % always start without velocity
    
    Aineq = []; Bineq = []; % we have no linear inequality constraints
    Aeq = []; Beq = []; % we have no equality constraints
    
    LB = [-Inf, -Inf, -Inf];
    UB = [Inf, Inf, Inf];
    
    options = optimset('display','iter','diffmaxchange',1.1*1e-5,'diffminchange',1e-5,'MaxFunEvals',200000,'MaxIter',200000,'TolCon',0.1,'Display','notify');
    
    [k,~] = ...
        fmincon(@jumpOffCost,k0,Aineq,Bineq,Aeq,Beq,LB,UB,@jumpOffNonl,options,td_LO,dld_LO,linit,dlinit);
    
    k11 = k(1);
    k12 = k(2);
    k13 = k(3);
    k14 = k(4);
end