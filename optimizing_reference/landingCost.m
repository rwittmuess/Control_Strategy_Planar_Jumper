function cost = landingCost(k,tl_min,tl_end,lF,dlF)
    k31 = k(1);
    k32 = k(2);
    k33 = k(3);
    
    cost = k31^2 + 0.1* k32^2 + k33^2;
end