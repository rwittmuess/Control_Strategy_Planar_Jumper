function cost = jumpOffCost(k, td_LO,dld_LO,linit,dlinit)
    k11 = k(1);
    k12 = k(2);
    k13 = k(3);
    k14 = k(4);

    cost = k11^2 + k12^2 + k13^2 + k14^2;
end