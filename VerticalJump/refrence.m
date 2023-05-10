syms ts tLO tLI real
% ts: time
% tLO: actual time at lift-off
% tLI: actual time at landing instance

tstar = ts - tLO;
t2star = ts - tLI;

t1s = [ts;tLO];
t2s = [ts;tLI];

% parameter take-off trajectory
k11 = 6.32;
k12 = 1.8;
k13 = -1.17;
k14 = 6.67;

% parameter flight trajectory
k21 = 0.13;
k22 = 25;
k23 = 0.01;

% parameter landing trajectory
k31 = 0.1;
k32 = 2;
k33 = 0.58;

% desired parameters
td_LO = 1; % time of lift-off
zd_Fmax = 0.12; % max foot hight in ther air
dld_LO = 0.4; % speed of CoM at lift-off
tz_Fmax = 0.075; % time after which max foot hight is reached
tl_min = 0.2; % something for landing instance

% reference during stance phase of the CoM
lref_LO = simplify(k11 * tanh(k12 * (ts - td_LO)) - 0.5*g*ts^2 + k13*ts + k14);
dlref_LO = simplify(diff(lref_LO,ts));

% reference during flight phase of the foot
zFref = simplify(k21*1/(cosh(k22*(tstar-tz_Fmax))^2)-k23);
dzFref = simplify(diff(zFref,ts));

pCOM_gen

% reference druing land phase
lref_LI = simplify(-k31*1/(cosh(k32*(t2star - tl_min))^2)-k33);
dlref_LI = simplify(diff(lref_LI,ts));

if ~exist('./gen')
    mkdir('./gen')
end
addpath('./gen')

matlabFunction(lref_LO,  'File',  'gen/lref_LO_gen', 'Vars', {ts});
matlabFunction(dlref_LO, 'File', 'gen/dlref_LO_gen', 'Vars', {ts});

matlabFunction(zFref,  'File', 'gen/zFref_gen',  'Vars', {t1s});
matlabFunction(dzFref, 'File', 'gen/dzFref_gen', 'Vars', {t1s});

matlabFunction(lref_LI,  'File', 'gen/lref_LI_gen',  'Vars', {t2s});
matlabFunction(dlref_LI,  'File', 'gen/dlref_LI_gen', 'Vars', {t2s});
