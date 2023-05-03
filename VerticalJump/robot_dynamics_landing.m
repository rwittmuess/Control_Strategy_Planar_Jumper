function ds = robot_dynamics_landing(t,s)
    % in s, each variable is a row
    % gains for sliding surface
    lambda1 = 20;
    lambda2 = 20;
    lambda3 = 20;
%     print(s)

    % control parameter
    M1 = 300;
    M2 = 300;
    M3 = 300;

    sz = size(s);
    ds = zeros(10,sz(2));
   

    D = D_gen(s);
    Cq = Cq_gen(s);
    G = G_gen(s);
    
    Jy = Jy_gen(s(1:5));

    Phi = Jy/D(1:3,1:3); % Jy * inv(D11)
    
    l = l_gen(s(1:5));
    dl = dl_gen(s);
    lref_LO = lref_LO_gen(t);
    dlref_LO = dlref_LO_gen(t);
    lt = l - lref_LO;
    dlt = dl - dlref_LO;

    theta = theta_gen(s(1:5));
    dtheta = dtheta_gen(s);

    theta3 = theta3_gen(s(1:5));
    dtheta3 = dtheta3_gen(s);

    sigma = [dlt + lambda1*lt;...
             dtheta + lambda2*theta;...
             dtheta3 + lambda3*theta3];

    tau = -inv(Phi) * [M1 * sign(sigma(1)); M2 * sign(sigma(2)); M3 * sign(sigma(3))];

    F0 = Cq(4:5) + G(4:5) + D(4:5,1:3) * (D(1:3,1:3)\(tau - Cq(1:3) - G(1:3)));

    F = [tau;F0];

    ds(1:5) = s(6:10);
    ds(6:10) = D\(F-Cq-G);
   
end