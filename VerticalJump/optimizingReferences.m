% desired values
td_LO = 1; % time of lift-off
zd_Fmax = 0.12; % max foot hight in the air
dld_LO = 0.4; % speed of CoM at lift-off
tz_Fmax = 0.075; % time after which max foot hight is reached
tl_min = 0.2; % something for landing instance

g = 9.81;

% h_max = vo^2/2g

tstar = ts - tLO;
t2star = ts - tLI;


lref_LO = k11 * tanh(k12 * (ts - td_LO)) - 0.5*g*ts^2 + k13*ts + k14;

zFref = k21*1/(cosh(k22*(tstar-tz_Fmax))^2)-k23;

lref_LI = -k31*1/(cosh(k32*(t2star - tl_min))^2)-k33;