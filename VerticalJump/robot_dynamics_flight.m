function ds = robot_dynamics_flight(t,s,LOv,td_LO,tz_Fmax)

    % persistent lambda1 lambda2 M1 M2 td_LO

    % controller for lref_F
    % compute reference with kinematics of zCOM not from state s
    % gains for sliding surface
    lambda1 = 20;
    lambda2 = 10;

    % control parameter
    M1 = 100;%300
    M2 = 100;

%     td_LO = 1; % time of lift-off; later take actually lift-off time, if I can manage to make lift off event depended on acceleraion for instance


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
    %% lref_F
    lref_F = lref_F_gen([t;td_LO;tz_Fmax;LOv]);
    %% dlref_F
    dlref_F = dlref_F_gen([t;td_LO;tz_Fmax;LOv]);
    %% lt
    lt = l - lref_F;
    %% dlt
    dlt = dl - dlref_F;
    %% theta
    theta = theta_gen(s(1:5));
    %% dtheta
    dtheta = dtheta_gen(s);
    %% vertical trunk is discarded (theta3 = 0 && dtheta3 = 0)

    
    %% Calculating
    Phi = Jy/D(1:3,1:3);
   
    sigma = [dlt + lambda1*lt;... % "-" tried
             dtheta + lambda2*theta];
    
    tau = -Phi(1:2,2:3)\(sign(sigma).*[M1;M2]);

    F = [0;tau;0;0]; %F0 = [0;0]

    % ds = zeros(10,1);
    % ds(1:5,1) = s(6:10);
    % ds(6:10,1) = D\(F-Cq-G);
    ds = [s(6:10);D\(F-Cq-G)];

end