function ds = robot_dynamics_ground(t,s)


    %% Setting constants

    % persistent lambda1 lambda2 lambda3 M1 M2 M3

    % in s, each variable is a row
    % gains for sliding surface
    lambda1 = 50;
    lambda2 = 20;
    lambda3 = 20;

    % control parameter
    M1 = 500;
    M2 = 400;
    M3 = 400; 
        
   
    %% D
    D = D_gen(s);  
    %% Cq
    Cq = Cq_gen(s);
    %% G
    G = G_gen(s);
    %% Jy
    Jy = Jy_gen(s(1:3));
    %% l
    l = l_gen(s(1:5));
    %% dl
    dl = dl_gen(s);
    %% lref_LO
    lref_LO = lref_LO_gen(t);
    %% dlref_LO
    dlref_LO = dlref_LO_gen(t);
    %% lt
    lt = l - lref_LO;
    %% dlt
    dlt = dl - dlref_LO;
    %% theta
    theta = theta_gen(s(1:5));
    %% dtheta
    dtheta = dtheta_gen(s);
    %% theta3
    theta3 = theta3_gen(s(1:5));
    %% dtheta3
    dtheta3 = dtheta3_gen(s);
    %% Calculating
    Phi = Jy/D(1:3,1:3);

    sigma = [dlt + lambda1*lt;...
             dtheta + lambda2*theta;...
             dtheta3 + lambda3*theta3];

    tau = -Phi\(sign(sigma).*[M1;M2;M3]);

    F0 = Cq(4:5) + G(4:5) + D(4:5,1:3) * (D(1:3,1:3)\(tau - Cq(1:3) - G(1:3)));

    F = [tau;F0];

    % ds = zeros(10,1);
    % ds(1:5) = s(6:10);
    % ds(6:10) = D\(F-Cq-G);
    ds = [s(6:10);D\(F-Cq-G)];

end