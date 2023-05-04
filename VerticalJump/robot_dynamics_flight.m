function ds = robot_dynamics_flight(t,s,LOv)

    % persistent lambda1 lambda2 M1 M2 td_LO

    % controller for lref_F
    % compute reference with kinematics of zCOM not from state s
    % gains for sliding surface
    lambda1 = 20;
    lambda2 = 10;

    % control parameter
    M1 = 100;%300
    M2 = 100;

    td_LO = 1; % time of lift-off; later take actually lift-off time, if I can manage to make lift off event depended on acceleraion for instance

    %% D
    % D = D_gen(s);  
    q1 = s(1);
    q2 = s(2);
    q3 = s(3);
    t2 = cos(q1);
    t3 = cos(q2);
    t4 = cos(q3);
    t5 = sin(q1);
    t6 = q1+q2;
    t7 = q2+q3;
    t8 = cos(t6);
    t9 = cos(t7);
    t10 = q3+t6;
    t11 = sin(t6);
    t14 = t3.*(3.6e+1./2.5e+1);
    t17 = t4.*(9.9e+1./5.0e+1);
    t18 = t2.*(2.69e+2./5.0e+1);
    t19 = t5.*(2.69e+2./5.0e+1);
    t21 = t4.*(9.9e+1./1.0e+2);
    t12 = cos(t10);
    t13 = sin(t10);
    t15 = t8.*(2.4e+1./5.0);
    t16 = t11.*(2.4e+1./5.0);
    t22 = -t19;
    t25 = t9.*(9.9e+1./1.0e+2);
    t27 = t21+7.63e+2./5.0e+2;
    t20 = -t16;
    t23 = t12.*(3.3e+1./1.0e+1);
    t24 = t13.*(3.3e+1./1.0e+1);
    t31 = t25+t27;
    t33 = t14+t17+t25+2.971;
    t26 = -t24;
    t28 = t15+t23;
    t29 = t20+t26;
    t30 = t18+t28;
    t32 = t22+t29;
    D = reshape([t3.*(7.2e+1./2.5e+1)+t9.*(9.9e+1./5.0e+1)+t17+4.5808,t33,t31,t32,t30,t33,t17+2.971,t27,t29,t28,t31,t27,7.63e+2./5.0e+2,t26,t23,t32,t29,t26,7.5e+1./4.0,0.0,t30,t28,t23,0.0,7.5e+1./4.0],[5,5]);
    %% Cq
    % Cq = Cq_gen(s);
    dq1 = s(6);
    dq2 = s(7);
    dq3 = s(8);
    q1 = s(1);
    q2 = s(2);
    q3 = s(3);
    t2 = sin(q2);
    t3 = sin(q3);
    t4 = q1+q2;
    t5 = q2+q3;
    t6 = dq1.^2;
    t7 = dq2.^2;
    t8 = dq3.^2;
    t9 = cos(t4);
    t10 = q3+t4;
    t11 = sin(t4);
    t12 = sin(t5);
    t15 = dq1.*dq3.*t3.*(9.9e+1./5.0e+1);
    t16 = dq2.*dq3.*t3.*(9.9e+1./5.0e+1);
    t19 = t3.*t8.*(9.9e+1./1.0e+2);
    t13 = cos(t10);
    t14 = sin(t10);
    t17 = -t15;
    t18 = -t16;
    t20 = -t19;
    t21 = t6.*t12.*(9.9e+1./1.0e+2);
    mt1 = [t17+t18+t20-t2.*t7.*(3.6e+1./2.5e+1)-t7.*t12.*(9.9e+1./1.0e+2)-t8.*t12.*(9.9e+1./1.0e+2)-dq1.*dq2.*t2.*(7.2e+1./2.5e+1)-dq1.*dq2.*t12.*(9.9e+1./5.0e+1)-dq1.*dq3.*t12.*(9.9e+1./5.0e+1)-dq2.*dq3.*t12.*(9.9e+1./5.0e+1);t17+t18+t20+t21+t2.*t6.*(3.6e+1./2.5e+1);t21+t3.*t6.*(9.9e+1./1.0e+2)+t3.*t7.*(9.9e+1./1.0e+2)+dq1.*dq2.*t3.*(9.9e+1./5.0e+1)];
    mt2 = [t6.*t9.*(-2.4e+1./5.0)-t7.*t9.*(2.4e+1./5.0)-t6.*t13.*(3.3e+1./1.0e+1)-t7.*t13.*(3.3e+1./1.0e+1)-t8.*t13.*(3.3e+1./1.0e+1)-t6.*cos(q1).*(2.69e+2./5.0e+1)-dq1.*dq2.*t9.*(4.8e+1./5.0)-dq1.*dq2.*t13.*(3.3e+1./5.0)-dq1.*dq3.*t13.*(3.3e+1./5.0)-dq2.*dq3.*t13.*(3.3e+1./5.0);t6.*t11.*(-2.4e+1./5.0)-t7.*t11.*(2.4e+1./5.0)-t6.*t14.*(3.3e+1./1.0e+1)-t7.*t14.*(3.3e+1./1.0e+1)-t8.*t14.*(3.3e+1./1.0e+1)-t6.*sin(q1).*(2.69e+2./5.0e+1)-dq1.*dq2.*t11.*(4.8e+1./5.0)-dq1.*dq2.*t14.*(3.3e+1./5.0)-dq1.*dq3.*t14.*(3.3e+1./5.0)-dq2.*dq3.*t14.*(3.3e+1./5.0)];
    Cq = [mt1;mt2];
    %% G
    % G = G_gen(s);
    q1 = s(1);
    q2 = s(2);
    q3 = s(3);
    t2 = q1+q2;
    t3 = q3+t2;
    t4 = sin(t2);
    t5 = sin(t3);
    t6 = t4.*4.7088e+1;
    t7 = -t6;
    t8 = t5.*3.2373e+1;
    t9 = -t8;
    G = [t7+t9-sin(q1).*5.27778e+1;t7+t9;t9;1.839375e+2;0.0];
    %% Jy
    % Jy = Jy_gen(s(1:3));
    q1 = s(1);
    q2 = s(2);
    q3 = s(3);
    t2 = cos(q1);
    t3 = cos(q2);
    t4 = cos(q3);
    t5 = sin(q1);
    t6 = q1+q2;
    t7 = q2+q3;
    t15 = sqrt(2.0);
    t8 = cos(t6);
    t9 = cos(t7);
    t10 = q3+t6;
    t11 = sin(t6);
    t12 = sin(t7);
    t16 = t2.*2.69e+2;
    t17 = t5.*2.69e+2;
    t20 = t4.*3.96e+4;
    t21 = t3.*6.456e+4;
    t13 = cos(t10);
    t14 = sin(t10);
    t18 = t8.*2.4e+2;
    t19 = t11.*2.4e+2;
    t24 = t9.*4.4385e+4;
    t25 = t12.*4.4385e+4;
    t22 = t13.*1.65e+2;
    t23 = t14.*1.65e+2;
    t31 = t20+t21+t24+7.8593e+4;
    t26 = t16+t18+t22;
    t27 = t17+t19+t23;
    t32 = 1.0./sqrt(t31);
    t28 = t27.^2;
    t29 = 1.0./t26;
    t30 = t29.^2;
    t33 = t28.*t30;
    t34 = t33+1.0;
    t35 = 1.0./t34;
    Jy = reshape([0.0,1.0,1.0,t15.*t32.*(t25+sin(q2).*6.456e+4).*(-5.333333333333333e-4),t35.*(t29.*(t18+t22)+t27.*t30.*(t19+t23)),1.0,t15.*t32.*(t25+sin(q3).*3.96e+4).*(-5.333333333333333e-4),t35.*(t22.*t29+t23.*t27.*t30),1.0],[3,3]);    
    %% l
    % l = l_gen(s(1:5));
    q2 = s(2);
    q3 = s(3);
    l = sqrt(2.0).*sqrt(cos(q2+q3).*4.4385e+4+cos(q2).*6.456e+4+cos(q3).*3.96e+4+7.8593e+4).*1.066666666666667e-3;
    %% dl
    % dl = dl_gen(s);
    dq2 = s(7);
    dq3 = s(8);
    q2 = s(2);
    q3 = s(3);
    t2 = q2+q3;
    t3 = sin(t2);
    dl = sqrt(2.0).*(dq2.*t3.*2.959e+3+dq3.*t3.*2.959e+3+dq2.*sin(q2).*4.304e+3+dq3.*sin(q3).*2.64e+3).*1.0./sqrt(cos(q2).*6.456e+4+cos(q3).*3.96e+4+cos(t2).*4.4385e+4+7.8593e+4).*(-1.0./1.25e+2);
    %% lref_F
    % lref_F = lref_F_gen([t;td_LO;LOv]);
    dl_LO = LOv(2);
    l_LO = LOv(1);
    tLO = td_LO;
    ts = t;
    t2 = -ts;
    t3 = t2+tLO;
    lref_F = l_LO-dl_LO.*t3-1.0./cosh(tLO.*2.5e+1-ts.*2.5e+1+1.5e+1./8.0).^2.*(1.3e+1./1.0e+2)-t3.^2.*(9.81e+2./2.0e+2)+1.0./1.0e+2;
    %% dlref_F
    % dlref_F = dlref_F_gen([t;td_LO;LOv]);
    dl_LO = LOv(2);
    tLO = td_LO;
    ts = t;
    t2 = tLO.*2.5e+1;
    t3 = ts.*2.5e+1;
    t4 = -t3;
    t5 = t2+t4+1.5e+1./8.0;
    dlref_F = dl_LO+tLO.*(9.81e+2./1.0e+2)-ts.*(9.81e+2./1.0e+2)-1.0./cosh(t5).^3.*sinh(t5).*(1.3e+1./2.0);
    %% lt
    lt = l - lref_F;
    %% dlt
    dlt = dl - dlref_F;
    %%
    %% theta
    % theta = theta_gen(s(1:5));
    q1 = s(1);
    q2 = s(2);
    q3 = s(3);
    t2 = q1+q2;
    t3 = q3+t2;
    theta = atan((sin(q1).*2.69e+2+sin(t2).*2.4e+2+sin(t3).*1.65e+2)./(cos(q1).*2.69e+2+cos(t2).*2.4e+2+cos(t3).*1.65e+2));
    %% dtheta
    % dtheta = dtheta_gen(s);
    dq1 = s(6);
    dq2 = s(7);
    dq3 = s(8);
    q2 = s(2);
    q3 = s(3);
    t2 = cos(q2);
    t3 = cos(q3);
    t4 = q2+q3;
    t5 = cos(t4);
    dtheta = (dq1.*1.57186e+5+dq2.*8.4825e+4+dq3.*2.7225e+4+dq1.*t2.*1.2912e+5+dq1.*t3.*7.92e+4+dq2.*t2.*6.456e+4+dq2.*t3.*7.92e+4+dq1.*t5.*8.877e+4+dq3.*t3.*3.96e+4+dq2.*t5.*4.4385e+4+dq3.*t5.*4.4385e+4)./(t2.*1.2912e+5+t3.*7.92e+4+t5.*8.877e+4+1.57186e+5);
    %% vertical trunk is discarded (theta3 = 0 && dtheta3 = 0)

    %% Calculating

    % Phi = Jy/D(1:3,1:3);
    % D33_inv = D(1:3,1:3)\eye(3);
    % Phi = Jy*D33_inv;
    D33_inv = D(1:3,1:3)\eye(3);
    D_inv = D\eye(5);
    Phi = Jy*D33_inv;
    
    sigma = [dlt + lambda1*lt;... % "-" tried
             dtheta + lambda2*theta];
    
    tau = -Phi(1:2,2:3)\(sign(sigma).*[M1;M2]);

    F = [0;tau;0;0]; %F0 = [0;0]

    % ds = zeros(10,1);
    % ds(1:5,1) = s(6:10);
    % ds(6:10,1) = D\(F-Cq-G);
    ds = [s(6:10);D_inv*(F-Cq-G)];

end