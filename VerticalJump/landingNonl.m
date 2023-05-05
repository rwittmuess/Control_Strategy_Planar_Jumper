function [c,ceq] = landingNonl(x, tl_min, linit, lend)
    ceq = [-x(1)*1/(cosh(2*(0 - tl_min))^2)-x(2) - linit;...
         -x(1)*1/(cosh(2*(3 - tl_min))^2)-x(2) - lend];
%     linit = -x(1)*1/(cosh(x(2)*(0 - tl_min))^2)-x(3);
%     lend = -x(1)*1/(cosh(x(2)*(3*tl_min - tl_min))^2)-x(3);
    
    j=0;
    for i = 0:0.1:3
        j = j+1;
        c(j) = -(-x(1)*1/(cosh(2*(i*tl_min - tl_min))^2)-x(2)) + 0.3;
        j = j+1;
        c(j) = -x(1)*1/(cosh(2*(i*tl_min - tl_min))^2)-x(2) - lend;
    end
%     for i = 0:0.1:3*tl_min
%         0.3 <= -x(1)*1/(cosh(x(2)*(i*tl_min - tl_min))^2)-x(3);
%         lend >= -x(1)*1/(cosh(x(2)*(i*tl_min - tl_min))^2)-x(3);
%     end