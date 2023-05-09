function D_inv = D_inv_gen(in1)
%D_inv_gen
%    D_inv = D_inv_gen(IN1)

%    This function was generated by the Symbolic Math Toolbox version 9.3.
%    08-May-2023 16:25:37

q1 = in1(1,:);
q2 = in1(2,:);
q3 = in1(3,:);
t2 = cos(q1);
t3 = cos(q2);
t4 = cos(q3);
t5 = sin(q1);
t6 = sin(q2);
t7 = sin(q3);
t8 = q1+q2;
t9 = q2+q3;
t10 = q1.*2.0;
t11 = q2.*2.0;
t12 = q3.*2.0;
t21 = -q1;
t22 = -q2;
t23 = -q3;
t13 = cos(t10);
t14 = cos(t11);
t15 = cos(t12);
t16 = sin(t10);
t17 = sin(t12);
t18 = cos(t8);
t19 = cos(t9);
t20 = sin(t8);
t24 = -t12;
t25 = q2+t8;
t26 = q1+t12;
t27 = q3+t9;
t28 = q2+t9;
t33 = t8+t12;
t36 = q1+t22;
t38 = q2+t23;
t39 = t8.*2.0;
t40 = t10+t12;
t41 = t9.*2.0;
t56 = t3.*5.01361e+5;
t63 = t4.*1.104279e+6;
t64 = t3.*3.509527e+6;
t65 = t3.*7.520415e+6;
t74 = t2.*t3.*3.223035e+6;
t75 = t3.*t5.*3.223035e+6;
t76 = t2.*2.25016717e+8;
t77 = t5.*2.25016717e+8;
t80 = t2.*t3.*t4.*1.2375e+4;
t81 = t3.*t4.*t5.*1.2375e+4;
t84 = t2.*t6.*4.9926551e+7;
t85 = t5.*t6.*4.9926551e+7;
t93 = t2.*1.125083585e+9;
t94 = t5.*1.125083585e+9;
t97 = t2.*t4.*t6.*1.94755e+5;
t98 = t4.*t5.*t6.*1.94755e+5;
t99 = t5.*t6.*t7.*5.35095e+5;
t113 = t2.*t3.*t7.*6.103075e+6;
t114 = t2.*t6.*t7.*5.35095e+5;
t115 = t3.*t5.*t7.*6.103075e+6;
t29 = cos(t25);
t30 = cos(t26);
t31 = cos(t27);
t32 = cos(t28);
t34 = sin(t25);
t35 = sin(t26);
t37 = q1+t24;
t42 = cos(t36);
t44 = cos(t38);
t45 = sin(t36);
t47 = cos(t33);
t48 = sin(t33);
t49 = t10+t24;
t50 = cos(t39);
t51 = cos(t40);
t52 = cos(t41);
t53 = t8+t27;
t54 = sin(t39);
t55 = sin(t40);
t58 = t21+t27;
t62 = t19.*4.2581e+4;
t66 = t8+t33;
t70 = -t64;
t73 = t19.*6.38715e+5;
t79 = t15.*6.58845e+5;
t83 = -t74;
t86 = -t76;
t87 = -t77;
t95 = t15.*6.7022505e+7;
t96 = -t81;
t100 = t18.*1.63462306e+8;
t101 = -t84;
t102 = t20.*1.63462306e+8;
t103 = -t85;
t108 = -t93;
t109 = -t94;
t110 = t14.*2.22692064e+8;
t111 = t15.*2.92804875e+8;
t112 = t15.*4.69157535e+8;
t116 = t2.*t3.*t15.*1.79685e+5;
t117 = t3.*t5.*t15.*1.79685e+5;
t118 = t2.*t6.*t17.*1.79685e+5;
t123 = t5.*t6.*t17.*1.79685e+5;
t127 = t13.*4.096404028e+9;
t131 = t14.*1.43159184e+9;
t132 = t14.*1.558844448e+9;
t133 = -t114;
t139 = t2.*t6.*t15.*2.030985e+6;
t140 = t2.*t3.*t17.*2.030985e+6;
t142 = t5.*t6.*t15.*2.030985e+6;
t143 = t3.*t5.*t17.*2.030985e+6;
t146 = t16.*1.6385616112e+10;
t43 = cos(t37);
t46 = sin(t37);
t57 = cos(t49);
t59 = sin(t49);
t60 = cos(t53);
t61 = sin(t53);
t67 = cos(t58);
t68 = sin(t58);
t71 = cos(t66);
t72 = sin(t66);
t78 = t31.*2.7951e+4;
t89 = t44.*4.0656e+4;
t90 = t32.*1.81104e+5;
t91 = t31.*1.95657e+5;
t92 = t31.*4.19265e+5;
t106 = t30.*5.210865e+6;
t107 = t35.*5.210865e+6;
t119 = t44.*6.0984e+5;
t121 = t30.*2.6054325e+7;
t122 = t29.*3.1813152e+7;
t125 = t35.*2.6054325e+7;
t126 = t34.*3.1813152e+7;
t128 = t52.*1.24509e+5;
t129 = t47.*6.47955e+6;
t130 = t48.*6.47955e+6;
t138 = -t117;
t141 = -t118;
t145 = t29.*1.5906576e+8;
t148 = t34.*1.5906576e+8;
t149 = -t127;
t151 = t52.*6.22545e+5;
t152 = t52.*4.002075e+6;
t153 = t52.*4.357815e+6;
t156 = t42.*1.86023551e+8;
t157 = -t146;
t158 = t45.*1.86023551e+8;
t164 = t50.*8.1805248e+7;
t165 = t51.*8.242641e+7;
t166 = t54.*3.27220992e+8;
t167 = t55.*3.2970564e+8;
t88 = -t78;
t104 = -t90;
t105 = -t92;
t120 = t43.*5.210865e+6;
t124 = t46.*5.210865e+6;
t134 = t60.*8.8935e+4;
t135 = t60.*4.44675e+5;
t136 = t61.*8.8935e+4;
t137 = t61.*4.44675e+5;
t144 = t43.*2.6054325e+7;
t147 = t46.*2.6054325e+7;
t150 = -t128;
t154 = -t129;
t155 = -t130;
t159 = t67.*7.737345e+6;
t160 = t68.*7.737345e+6;
t161 = -t156;
t162 = -t158;
t163 = t71.*2.2869e+5;
t169 = t72.*9.1476e+5;
t170 = t57.*8.242641e+7;
t171 = t59.*3.2970564e+8;
t172 = t70+t79+t91-1.211317e+7;
t174 = t95+t110+t151-1.466098489e+9;
t176 = t112+t132+t153-1.0262689423e+10;
t189 = t75+t96+t97+t99+t101+t113+t123+t138+t139+t140;
t192 = t80+t83+t98+t103+t115+t116+t133+t141+t142+t143;
t168 = -t160;
t175 = 1.0./t174;
t177 = 1.0./t176;
t178 = t86+t106+t120+t122+t134;
t179 = t87+t107+t124+t126+t136;
t180 = t157+t166+t167+t169+t171;
t190 = t100+t108+t121+t135+t144+t145+t154+t159+t161;
t182 = t175.*(t56-t62+t88+t89).*1.5e+4;
t184 = t172.*t177.*1.5e+4;
t186 = t175.*t178.*(2.0e+2./7.0);
t187 = t175.*t179.*(2.0e+2./7.0);
t191 = t102+t109+t125+t137+t147+t148+t155+t162+t168;
t193 = (t175.*t180)./2.1e+1;
t195 = t175.*(t63+t65-t73+t104+t105+t119+t150+8.242874e+6).*1.0e+3;
t196 = t175.*t189.*4.0e+1;
t197 = t175.*t192.*4.0e+1;
t200 = t175.*t190.*(4.0e+1./7.0);
t183 = -t182;
t185 = -t184;
t188 = -t186;
t194 = -t193;
t198 = -t196;
t199 = -t197;
t201 = t175.*t191.*(4.0e+1./7.0);
t202 = -t201;
mt1 = [t177.*(t15.*1.31769e+5-2.422634e+6).*7.5e+4,t185,t183,t187,t188,t185,t177.*(t3.*-5.2642905e+7+t15.*4.9413375e+6+t31.*2.934855e+6+t52.*4.357815e+5-1.19698834e+8).*2.0e+3,t195,t202,t200,t183,t195,t175.*(t4.*-4.417116e+6+t14.*5.26848e+5+t32.*7.24416e+5+t52.*2.49018e+5-1.9786721e+7).*5.0e+2,t198,t199,t187,t202,t198,t177.*(-t111-t131+t149-t152+t163+t164+t165+t170+5.907129245e+9).*(-4.0./3.0),t194,t188,t200,t199,t194];
mt2 = [t177.*(t111+t131+t149+t152+t163+t164+t165+t170-5.907129245e+9).*(4.0./3.0)];
D_inv = reshape([mt1,mt2],5,5);
end
