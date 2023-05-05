function ds = robot_dynamics_landing(t,s,t_LI)
    

    %% Setting constants

    % in s, each variable is a row
    % gains for sliding surface
    lambda1 = 50;
    lambda2 = 20;
    lambda3 = 20;

    % control parameter
    M1 = 500;
    M2 = 400;
    M3 = 400; 

    sz = size(s);
    ds = zeros(10,sz(2));
   
    %% Import functions
    D = D_gen(s);
    Cq = Cq_gen(s);
    G = G_gen(s);
    
    Jy = Jy_gen(s(1:5));

    Phi = Jy/D(1:3,1:3); % Jy * inv(D11)
    
    l = l_gen(s(1:5));
    dl = dl_gen(s);
    lref_LI = lref_LI_gen([t;t_LI]);
    dlref_LI = dlref_LI_gen([t;t_LI]);
    lt = l - lref_LI;
    dlt = dl - dlref_LI;

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