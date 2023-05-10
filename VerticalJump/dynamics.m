function dynamics(  k11, k12, k13, k14,...
                    k21, k22, k23,...
                    k31, k32, k33,...
                    k41, k42, k43, k44)

syms q1 q2 q3 real
syms dq1 dq2 dq3 real
syms zF xF real
syms dzF dxF real
syms tau1 tau2 tau3 real
syms ddq1 ddq2 ddq3 ddzF ddxF real

q = [q1;q2;q3;zF;xF];

dq = [dq1;dq2;dq3;dzF;dxF];

ddq = [ddq1;ddq2;ddq3;ddzF;ddxF];

s = [q;dq];

tau = [tau1;tau2;tau3];

% 1: lower leg
% 2: upper leg
% 3: torso

l1 = 0.3;
l2 = 0.3;
l3 = 0.5; % I made this number up, not given by paper

lc1 = 0.16;
lc2 = 0.15;
lc3 = 0.22;

m1 = 1.75;
m2 = 2;
m3 = 15;

J1 = 0.035; % I made all inertias up, not given by paper
J2 = 0.05;
J3 = 0.8;

g = 9.81;

p1 = [xF + lc1*sin(q1);...
      zF + lc1*cos(q1)];
p2 = [xF + l1*sin(q1) + lc2*sin(q1+q2);...
      zF + l1*cos(q1) + lc2*cos(q1+q2)];
p3 = [xF + l1*sin(q1) + l2*sin(q1+q2) + lc3*sin(q1+q2+q3);...
      zF + l1*cos(q1) + l2*cos(q1+q2) + lc3*cos(q1+q2+q3)];

pKnee = [xF + l1*sin(q1);...
         zF + l1*cos(q1)];
pHip = [xF + l1*sin(q1) + l2*sin(q1+q2);...
        zF + l1*cos(q1) + l2*cos(q1+q2)];
pHead = [xF + l1*sin(q1) + l2*sin(q1+q2) + l3*sin(q1+q2+q3);...
         zF + l1*cos(q1) + l2*cos(q1+q2) + l3*cos(q1+q2+q3)];

% compute CoM of Robot
xCOM = (p1(1)*m1 + p2(1)*m2 + p3(1)*m3)/(m1+m2+m3);
zCOM = (p1(2)*m1 + p2(2)*m2 + p3(2)*m3)/(m1+m2+m3);
pCOM = [xCOM;zCOM];

dxCOM = simplify(jacobian(xCOM,q)*dq);
dzCOM = simplify(jacobian(zCOM,q)*dq);

if ~exist('./gen')
    mkdir('./gen')
end
addpath('./gen')

matlabFunction(pKnee, 'File', 'gen/pKnee_gen', 'Vars', {q});
matlabFunction(pHip,  'File', 'gen/pHip_gen',  'Vars', {q});
matlabFunction(pHead, 'File', 'gen/pHead_gen', 'Vars', {q});
matlabFunction(pCOM,  'File', 'gen/pCOM_gen',  'Vars', {q});
matlabFunction(dxCOM, 'File', 'gen/dxCOM_gen', 'Vars', {s});
matlabFunction(dzCOM, 'File', 'gen/dzCOM_gen', 'Vars', {s});
matlabFunction(zCOM, 'File', 'gen/zCOM_gen', 'Vars', {q});

dp1 = simplify(jacobian(p1, q)*dq);
dp2 = simplify(jacobian(p2, q)*dq);
dp3 = simplify(jacobian(p3, q)*dq);

dq1abs = dq1;
dq2abs = dq1 + dq2;
dq3abs = dq1 + dq2 + dq3;

KE1 = 0.5*m1*dp1(1)^2 + 0.5*m1*dp1(2)^2 + 0.5*J1*dq1abs^2;
KE2 = 0.5*m2*dp2(1)^2 + 0.5*m2*dp2(2)^2 + 0.5*J2*dq2abs^2;
KE3 = 0.5*m3*dp3(1)^2 + 0.5*m3*dp3(2)^2 + 0.5*J3*dq3abs^2;

KE = KE1 + KE2 + KE3;


PE1 = m1*g*p1(2);
PE2 = m2*g*p2(2);
PE3 = m3*g*p3(2);

PE = PE1 + PE2 + PE3;

qActuated = [q1;q2;q3];

[D, Cq, G, B] = LagrangianDynamics(KE, PE, q, dq, qActuated);

matlabFunction(D,  'File', 'gen/D_gen',   'Vars', {s});
matlabFunction(Cq, 'File', 'gen/Cq_gen',  'Vars', {s});
matlabFunction(G,  'File', 'gen/G_gen',   'Vars', {s});

l = simplify(sqrt((pCOM(1)-xF)^2 + (pCOM(2)-zF)^2));
theta = simplify(atan((pCOM(1)-xF)/(pCOM(2)-zF)));
theta3 = q1+q2+q3;

dl = simplify(jacobian(l, q)*dq); % not sure about this
dtheta = simplify(jacobian(theta, q)*dq); % not sure about this
dtheta3 = simplify(jacobian(theta3, q)*dq); % not sure about this

qq = [q1;q2;q3];

Jy = jacobian([l;theta;theta3],qq);

JyF = jacobian([zF;theta;theta3],qq);

matlabFunction(l,       'File', 'gen/l_gen',        'Vars', {q});
matlabFunction(dl,      'File', 'gen/dl_gen',       'Vars', {s});
% matlabFunction(ddl,     'File', 'gen/ddl_gen',      'Vars', {sq});
matlabFunction(theta,   'File', 'gen/theta_gen',    'Vars', {q});
matlabFunction(dtheta,  'File', 'gen/dtheta_gen',   'Vars', {s});
matlabFunction(theta3,  'File', 'gen/theta3_gen',   'Vars', {q});
matlabFunction(dtheta3, 'File', 'gen/dtheta3_gen',  'Vars', {s});
matlabFunction(Jy,      'File', 'gen/Jy_gen',       'Vars', {q});
matlabFunction(JyF,     'File', 'gen/JyF_gen',      'Vars', {q});

% pF = [xF;zF];
% 
% JF = simplify(jacobian(pF, q));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

syms ts tLI dl_LO l_LO td_LO tl_min tz_Fmax real
% ts: time
% tLO: actual time at lift-off
% tLI: actual time at landing instance

tstar = ts - td_LO;
t2star = ts - tLI;

t0s = [ts;td_LO];
t1s = [ts;td_LO;tz_Fmax];
t2s = [ts;tLI;tl_min];

LO = [l_LO;dl_LO];

t1sLO = [t1s;LO];

% reference during stance phase of the CoM
lref_LO = simplify(k11 * tanh(k12 * (ts - td_LO)) - 0.5*g*ts^2 + k13*ts + k14);
dlref_LO = simplify(diff(lref_LO,ts));

% reference during flight phase of the foot
zFref = simplify(k21*1/(cosh(k22*(tstar-tz_Fmax))^2)-k23);
dzFref = simplify(diff(zFref,ts));

% lref_F = zCOM - zFref;
% dlref_F = dzCOM - dzFref;

% other approach for reference l during flight phase
zCOM_kin = simplify(dl_LO*tstar - g/2 * tstar^2 + l_LO);
dzCOM_kin = simplify(diff(zCOM_kin,ts));

lref_F = zCOM_kin - zFref;
dlref_F = dzCOM_kin - dzFref;


% reference during landing phase
lref_LI = simplify(-k31*1/(cosh(k32*(t2star - tl_min))^2)-k33);
dlref_LI = simplify(diff(lref_LI,ts));

if ~exist('./gen')
    mkdir('./gen')
end
addpath('./gen')

matlabFunction(lref_LO,  'File',  'gen/lref_LO_gen', 'Vars', {t0s});
matlabFunction(dlref_LO, 'File', 'gen/dlref_LO_gen', 'Vars', {t0s});

matlabFunction(zFref,  'File', 'gen/zFref_gen',  'Vars', {t1s});
matlabFunction(dzFref, 'File', 'gen/dzFref_gen', 'Vars', {t1s});

matlabFunction(zCOM_kin,  'File', 'gen/zCOM_kin_gen',  'Vars', {t1sLO});
matlabFunction(dzCOM_kin, 'File', 'gen/dzCOM_kin_gen', 'Vars', {t1sLO});

matlabFunction(lref_F,  'File', 'gen/lref_F_gen',  'Vars', {t1sLO});
matlabFunction(dlref_F, 'File', 'gen/dlref_F_gen', 'Vars', {t1sLO});

matlabFunction(lref_LI,  'File', 'gen/lref_LI_gen',  'Vars', {t2s});
matlabFunction(dlref_LI,  'File', 'gen/dlref_LI_gen', 'Vars', {t2s});
