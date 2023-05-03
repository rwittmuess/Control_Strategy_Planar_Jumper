function ds = robot_dynamics_flightV3(t,s,LOv)
    % controller for lref_F
    % compute reference with kinematics of zCOM not from state s
    % gains for sliding surface
    lambda1 = 20;%50
    lambda2 = 10;
%     lambda3 = 10;

    % control parameter
    M1 = 100;%300
    M2 = 100;
%     M3 = 100;

    td_LO = 1; % time of lift-off; later take actually lift-off time, if I can manage to make lift off event depended on acceleraion for instance

    D = D_gen(s);
    Cq = Cq_gen(s);
    G = G_gen(s);
    
    Jy = Jy_gen(s(1:5));

    Phi = Jy/D(1:3,1:3); % Jy * inv(D11)
    
    l = l_gen(s(1:5));
    dl = dl_gen(s);
    lref_F = lref_F_gen([t;td_LO;LOv]);
    dlref_F = dlref_F_gen([t;td_LO;LOv]);
    lt = l - lref_F;
    dlt = dl - dlref_F;

    theta = theta_gen(s(1:5));
    dtheta = dtheta_gen(s);

    % vertical trunk is discarded (theta3 = 0)

    sigma = [dlt + lambda1*lt;... % hab es mal mit - probiert
             dtheta + lambda2*theta];

    tau = -Phi(1:2,2:3)\(sign(sigma).*[M1;M2]);

%     F0 = Cq(4:5) + G(4:5) + D(4:5,1:3) * (D(1:3,1:3)\(tau - Cq(1:3) - G(1:3)));
    F0 = [0;0];

    F = [0;tau;F0];

    ds = zeros(10,1);
    ds(1:5,1) = s(6:10);
    ds(6:10,1) = D\(F-Cq-G);
end