function cost = flightCost(k,tz_Fmax, zd_Fmax)
    k21 = k(1);
    k22 = k(2);
    k23 = k(3);

    cost = k21^2 + 0.00001*k22^2 + k23^2;
end