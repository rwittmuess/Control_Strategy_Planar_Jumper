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


    %% Simplifying expressions that are used frequently for better runtime
    q1 = s(1);
    q2 = s(2);
    q3 = s(3);
    dq1 = s(6);
    dq2 = s(7);
    dq3 = s(8);
    
    sq1 = sin(q1);
    sq2 = sin(q2);
    sq3 = sin(q3);
    cq1 = cos(q1);
    cq2 = cos(q2);
    cq3 = cos(q3);

    q12 = q1+q2;
    sq12 = sin(q12);
    cq12 = cos(q12);

    q23 = q2+q3;
    sq23 = sin(q23);
    cq23 = cos(q23);

    sqrt2 = sqrt(2.0);

    %% D
    % D = D_gen(s);  
    t2 = cq1;
    t3 = cq2;
    t4 = cq3;
    t5 = sq1;
    t6 = q12;
    t8 = cq12;
    t9 = cq23;
    t10 = q3+t6;
    t11 = sq12;
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
    t2 = sq2;
    t3 = sq3;
    t4 = q12;
    t6 = dq1.^2;
    t7 = dq2.^2;
    t8 = dq3.^2;
    t9 = cq12;
    t10 = q3+t4;
    t11 = sq12;
    t12 = sq23;
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
    mt2 = [t6.*t9.*(-2.4e+1./5.0)-t7.*t9.*(2.4e+1./5.0)-t6.*t13.*(3.3e+1./1.0e+1)-t7.*t13.*(3.3e+1./1.0e+1)-t8.*t13.*(3.3e+1./1.0e+1)-t6.*cq1.*(2.69e+2./5.0e+1)-dq1.*dq2.*t9.*(4.8e+1./5.0)-dq1.*dq2.*t13.*(3.3e+1./5.0)-dq1.*dq3.*t13.*(3.3e+1./5.0)-dq2.*dq3.*t13.*(3.3e+1./5.0);t6.*t11.*(-2.4e+1./5.0)-t7.*t11.*(2.4e+1./5.0)-t6.*t14.*(3.3e+1./1.0e+1)-t7.*t14.*(3.3e+1./1.0e+1)-t8.*t14.*(3.3e+1./1.0e+1)-t6.*sin(q1).*(2.69e+2./5.0e+1)-dq1.*dq2.*t11.*(4.8e+1./5.0)-dq1.*dq2.*t14.*(3.3e+1./5.0)-dq1.*dq3.*t14.*(3.3e+1./5.0)-dq2.*dq3.*t14.*(3.3e+1./5.0)];
    Cq = [mt1;mt2];
    %% G
    % G = G_gen(s);
    t2 = q12;
    t3 = q3+t2;
    t4 = sq12;
    t5 = sin(t3);
    t6 = t4.*4.7088e+1;
    t7 = -t6;
    t8 = t5.*3.2373e+1;
    t9 = -t8;
    G = [t7+t9-sin(q1).*5.27778e+1;t7+t9;t9;1.839375e+2;0.0];
    %% Jy
    % Jy = Jy_gen(s(1:3));
    t2 = cq1;
    t3 = cq2;
    t4 = cq3;
    t5 = sq1;
    t6 = q12;
    t15 = sqrt2;
    t8 = cq12;
    t9 = cq23;
    t10 = q3+t6;
    t11 = sq12;
    t12 = sq23;
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
    Jy = reshape([0.0,1.0,1.0,t15.*t32.*(t25+sq2.*6.456e+4).*(-5.333333333333333e-4),t35.*(t29.*(t18+t22)+t27.*t30.*(t19+t23)),1.0,t15.*t32.*(t25+sin(q3).*3.96e+4).*(-5.333333333333333e-4),t35.*(t22.*t29+t23.*t27.*t30),1.0],[3,3]);    
    %% l
    % l = l_gen(s(1:5));
    l = sqrt2.*sqrt(cos(q2+q3).*4.4385e+4+cq2.*6.456e+4+cq3.*3.96e+4+7.8593e+4).*1.066666666666667e-3;
    %% dl
    % dl = dl_gen(s);
    dl = sqrt2.*(dq2.*sq23.*2.959e+3+dq3.*sq23.*2.959e+3+dq2.*sq2.*4.304e+3+dq3.*sin(q3).*2.64e+3).*1.0./sqrt(cq2.*6.456e+4+cq3.*3.96e+4+cq23.*4.4385e+4+7.8593e+4).*(-1.0./1.25e+2);
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
    t2 = q12;
    t3 = q3+t2;
    theta = atan((sin(q1).*2.69e+2+sin(t2).*2.4e+2+sin(t3).*1.65e+2)./(cq1.*2.69e+2+cq12.*2.4e+2+cos(t3).*1.65e+2));
    %% dtheta
    % dtheta = dtheta_gen(s);
    t2 = cq2;
    t3 = cq3;
    t5 = cq23;
    dtheta = (dq1.*1.57186e+5+dq2.*8.4825e+4+dq3.*2.7225e+4+dq1.*t2.*1.2912e+5+dq1.*t3.*7.92e+4+dq2.*t2.*6.456e+4+dq2.*t3.*7.92e+4+dq1.*t5.*8.877e+4+dq3.*t3.*3.96e+4+dq2.*t5.*4.4385e+4+dq3.*t5.*4.4385e+4)./(t2.*1.2912e+5+t3.*7.92e+4+t5.*8.877e+4+1.57186e+5);
    %% vertical trunk is discarded (theta3 = 0 && dtheta3 = 0)

    % %% D_inv
    % t2 = cq1;
    % t3 = cq2;
    % t4 = cq3;
    % t5 = sq1;
    % t6 = sq2;
    % t7 = sq3;
    % t8 = q12;
    % t9 = q23;
    % t10 = q1.*2.0;
    % t11 = q2.*2.0;
    % t12 = q3.*2.0;
    % t21 = -q1;
    % t22 = -q2;
    % t23 = -q3;
    % t13 = cos(t10);
    % t14 = cos(t11);
    % t15 = cos(t12);
    % t16 = sin(t10);
    % t17 = sin(t12);
    % t18 = cos(t8);
    % t19 = cos(t9);
    % t20 = sin(t8);
    % t24 = -t12;
    % t25 = q2+t8;
    % t26 = q1+t12;
    % t27 = q3+t9;
    % t28 = q2+t9;
    % t33 = t8+t12;
    % t36 = q1+t22;
    % t38 = q2+t23;
    % t39 = t8.*2.0;
    % t40 = t10+t12;
    % t41 = t9.*2.0;
    % t56 = t3.*5.01361e+5;
    % t63 = t4.*1.104279e+6;
    % t64 = t3.*3.509527e+6;
    % t65 = t3.*7.520415e+6;
    % t74 = t2.*t3.*3.223035e+6;
    % t75 = t3.*t5.*3.223035e+6;
    % t76 = t2.*2.25016717e+8;
    % t77 = t5.*2.25016717e+8;
    % t80 = t2.*t3.*t4.*1.2375e+4;
    % t81 = t3.*t4.*t5.*1.2375e+4;
    % t84 = t2.*t6.*4.9926551e+7;
    % t85 = t5.*t6.*4.9926551e+7;
    % t93 = t2.*1.125083585e+9;
    % t94 = t5.*1.125083585e+9;
    % t97 = t2.*t4.*t6.*1.94755e+5;
    % t98 = t4.*t5.*t6.*1.94755e+5;
    % t99 = t5.*t6.*t7.*5.35095e+5;
    % t113 = t2.*t3.*t7.*6.103075e+6;
    % t114 = t2.*t6.*t7.*5.35095e+5;
    % t115 = t3.*t5.*t7.*6.103075e+6;
    % t29 = cos(t25);
    % t30 = cos(t26);
    % t31 = cos(t27);
    % t32 = cos(t28);
    % t34 = sin(t25);
    % t35 = sin(t26);
    % t37 = q1+t24;
    % t42 = cos(t36);
    % t44 = cos(t38);
    % t45 = sin(t36);
    % t47 = cos(t33);
    % t48 = sin(t33);
    % t49 = t10+t24;
    % t50 = cos(t39);
    % t51 = cos(t40);
    % t52 = cos(t41);
    % t53 = t8+t27;
    % t54 = sin(t39);
    % t55 = sin(t40);
    % t58 = t21+t27;
    % t62 = t19.*4.2581e+4;
    % t66 = t8+t33;
    % t70 = -t64;
    % t73 = t19.*6.38715e+5;
    % t79 = t15.*6.58845e+5;
    % t83 = -t74;
    % t86 = -t76;
    % t87 = -t77;
    % t95 = t15.*6.7022505e+7;
    % t96 = -t81;
    % t100 = t18.*1.63462306e+8;
    % t101 = -t84;
    % t102 = t20.*1.63462306e+8;
    % t103 = -t85;
    % t108 = -t93;
    % t109 = -t94;
    % t110 = t14.*2.22692064e+8;
    % t111 = t15.*2.92804875e+8;
    % t112 = t15.*4.69157535e+8;
    % t116 = t2.*t3.*t15.*1.79685e+5;
    % t117 = t3.*t5.*t15.*1.79685e+5;
    % t118 = t2.*t6.*t17.*1.79685e+5;
    % t123 = t5.*t6.*t17.*1.79685e+5;
    % t127 = t13.*4.096404028e+9;
    % t131 = t14.*1.43159184e+9;
    % t132 = t14.*1.558844448e+9;
    % t133 = -t114;
    % t139 = t2.*t6.*t15.*2.030985e+6;
    % t140 = t2.*t3.*t17.*2.030985e+6;
    % t142 = t5.*t6.*t15.*2.030985e+6;
    % t143 = t3.*t5.*t17.*2.030985e+6;
    % t146 = t16.*1.6385616112e+10;
    % t43 = cos(t37);
    % t46 = sin(t37);
    % t57 = cos(t49);
    % t59 = sin(t49);
    % t60 = cos(t53);
    % t61 = sin(t53);
    % t67 = cos(t58);
    % t68 = sin(t58);
    % t71 = cos(t66);
    % t72 = sin(t66);
    % t78 = t31.*2.7951e+4;
    % t89 = t44.*4.0656e+4;
    % t90 = t32.*1.81104e+5;
    % t91 = t31.*1.95657e+5;
    % t92 = t31.*4.19265e+5;
    % t106 = t30.*5.210865e+6;
    % t107 = t35.*5.210865e+6;
    % t119 = t44.*6.0984e+5;
    % t121 = t30.*2.6054325e+7;
    % t122 = t29.*3.1813152e+7;
    % t125 = t35.*2.6054325e+7;
    % t126 = t34.*3.1813152e+7;
    % t128 = t52.*1.24509e+5;
    % t129 = t47.*6.47955e+6;
    % t130 = t48.*6.47955e+6;
    % t138 = -t117;
    % t141 = -t118;
    % t145 = t29.*1.5906576e+8;
    % t148 = t34.*1.5906576e+8;
    % t149 = -t127;
    % t151 = t52.*6.22545e+5;
    % t152 = t52.*4.002075e+6;
    % t153 = t52.*4.357815e+6;
    % t156 = t42.*1.86023551e+8;
    % t157 = -t146;
    % t158 = t45.*1.86023551e+8;
    % t164 = t50.*8.1805248e+7;
    % t165 = t51.*8.242641e+7;
    % t166 = t54.*3.27220992e+8;
    % t167 = t55.*3.2970564e+8;
    % t88 = -t78;
    % t104 = -t90;
    % t105 = -t92;
    % t120 = t43.*5.210865e+6;
    % t124 = t46.*5.210865e+6;
    % t134 = t60.*8.8935e+4;
    % t135 = t60.*4.44675e+5;
    % t136 = t61.*8.8935e+4;
    % t137 = t61.*4.44675e+5;
    % t144 = t43.*2.6054325e+7;
    % t147 = t46.*2.6054325e+7;
    % t150 = -t128;
    % t154 = -t129;
    % t155 = -t130;
    % t159 = t67.*7.737345e+6;
    % t160 = t68.*7.737345e+6;
    % t161 = -t156;
    % t162 = -t158;
    % t163 = t71.*2.2869e+5;
    % t169 = t72.*9.1476e+5;
    % t170 = t57.*8.242641e+7;
    % t171 = t59.*3.2970564e+8;
    % t172 = t70+t79+t91-1.211317e+7;
    % t174 = t95+t110+t151-1.466098489e+9;
    % t176 = t112+t132+t153-1.0262689423e+10;
    % t189 = t75+t96+t97+t99+t101+t113+t123+t138+t139+t140;
    % t192 = t80+t83+t98+t103+t115+t116+t133+t141+t142+t143;
    % t168 = -t160;
    % t175 = 1.0./t174;
    % t177 = 1.0./t176;
    % t178 = t86+t106+t120+t122+t134;
    % t179 = t87+t107+t124+t126+t136;
    % t180 = t157+t166+t167+t169+t171;
    % t190 = t100+t108+t121+t135+t144+t145+t154+t159+t161;
    % t182 = t175.*(t56-t62+t88+t89).*1.5e+4;
    % t184 = t172.*t177.*1.5e+4;
    % t186 = t175.*t178.*(2.0e+2./7.0);
    % t187 = t175.*t179.*(2.0e+2./7.0);
    % t191 = t102+t109+t125+t137+t147+t148+t155+t162+t168;
    % t193 = (t175.*t180)./2.1e+1;
    % t195 = t175.*(t63+t65-t73+t104+t105+t119+t150+8.242874e+6).*1.0e+3;
    % t196 = t175.*t189.*4.0e+1;
    % t197 = t175.*t192.*4.0e+1;
    % t200 = t175.*t190.*(4.0e+1./7.0);
    % t183 = -t182;
    % t185 = -t184;
    % t188 = -t186;
    % t194 = -t193;
    % t198 = -t196;
    % t199 = -t197;
    % t201 = t175.*t191.*(4.0e+1./7.0);
    % t202 = -t201;
    % mt1 = [t177.*(t15.*1.31769e+5-2.422634e+6).*7.5e+4,t185,t183,t187,t188,t185,t177.*(t3.*-5.2642905e+7+t15.*4.9413375e+6+t31.*2.934855e+6+t52.*4.357815e+5-1.19698834e+8).*2.0e+3,t195,t202,t200,t183,t195,t175.*(t4.*-4.417116e+6+t14.*5.26848e+5+t32.*7.24416e+5+t52.*2.49018e+5-1.9786721e+7).*5.0e+2,t198,t199,t187,t202,t198,t177.*(-t111-t131+t149-t152+t163+t164+t165+t170+5.907129245e+9).*(-4.0./3.0),t194,t188,t200,t199,t194];
    % mt2 = [t177.*(t111+t131+t149+t152+t163+t164+t165+t170-5.907129245e+9).*(4.0./3.0)];
    % D_inv = reshape([mt1,mt2],5,5);
    % %% D33_inv
    % t2 = cq2;
    % t3 = cq3;
    % t4 = q23;
    % t5 = q2.*2.0;
    % t6 = q3.*2.0;
    % t10 = -q3;
    % t7 = cos(t5);
    % t8 = cos(t6);
    % t9 = cos(t4);
    % t11 = q3+t4;
    % t12 = q2+t4;
    % t15 = q2+t10;
    % t16 = t4.*2.0;
    % t18 = t2.*1.8971e+4;
    % t21 = t2.*1.70739e+5;
    % t24 = t3.*7.34085e+5;
    % t25 = t2.*1.422825e+6;
    % t13 = cos(t11);
    % t14 = cos(t12);
    % t17 = cos(t15);
    % t19 = cos(t16);
    % t20 = t9.*7.975e+3;
    % t23 = -t21;
    % t26 = t8.*4.9005e+4;
    % t28 = t9.*5.98125e+5;
    % t34 = t8.*1.3868415e+7;
    % t37 = t7.*1.460808e+8;
    % t27 = t13.*5.445e+3;
    % t30 = t17.*7.92e+3;
    % t31 = t13.*4.9005e+4;
    % t33 = t13.*4.08375e+5;
    % t36 = t14.*5.94e+5;
    % t38 = t17.*5.94e+5;
    % t40 = t19.*4.08375e+5;
    % t29 = -t27;
    % t41 = -t40;
    % t42 = t23+t26+t31-1.71502e+5;
    % t44 = t34+t37+t40-1.96038691e+8;
    % t45 = 1.0./t44;
    % t47 = t42.*t45.*1.666666666666667e+3;
    % t49 = t45.*(t18-t20+t29+t30).*1.5e+4;
    % t51 = t45.*(t24+t25-t28-t33-t36+t38+t41+1.638754e+6).*2.0e+2;
    % t48 = -t47;
    % t50 = -t49;
    % D33_inv = reshape([t45.*(t26-1.71502e+5).*1.666666666666667e+3,t48,t50,t48,t45.*(t2.*-4.268475e+6+t8.*6.125625e+5+t13.*1.225125e+6+t19.*6.125625e+5-4.601906e+6).*(4.0e+2./3.0),t51,t50,t51,t45.*(t3.*-2.93634e+6+t7.*1.728e+6+t14.*2.376e+6+t19.*8.1675e+5-5.426443e+6).*1.0e+2],[3,3]);


    %% Calculating

    % Phi = Jy/D(1:3,1:3);
    % D33_inv = D(1:3,1:3)\eye(3);
    % Phi = Jy*D33_inv;
    % D33_inv = D(1:3,1:3)\eye(3);
    % D_inv = D\eye(5);
    Phi = Jy/D(1:3,1:3);
    % Phi_inv = Phi(1:2,2:3)\eye(2);

    sigma = [dlt + lambda1*lt;... % "-" tried
             dtheta + lambda2*theta];
    
    tau = -Phi(1:2,2:3)\(sign(sigma).*[M1;M2]);

    F = [0;tau;0;0]; %F0 = [0;0]

    % ds = zeros(10,1);
    % ds(1:5,1) = s(6:10);
    % ds(6:10,1) = D\(F-Cq-G);
    ds = [s(6:10);D\(F-Cq-G)];

end