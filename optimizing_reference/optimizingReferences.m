function [  k11, k12, k13, k14,...
            k21, k22, k23,...
            k31, k32, k33,...
            k41, k42, k43, k44] = optimizingReferences(td_LO, dld_LO, zd_Fmax, tz_Fmax, tl_min, tl_end)
%% defining constantss
g = 9.81;

%% Jump Off
[k11, k12, k13, k14] = optJumpOff(td_LO, dld_LO);

t = 0:0.01:td_LO;
lref = k11 .* tanh(k12 * (t - td_LO)) - 0.5*g*t.^2 + k13*t + k14;
dlref = k11 * k12 .* sech(k12*(t-td_LO)).^2 - g*t + k13;
% ddlref = -k11 * k12 .* sech(k12 * (t-td_LO)).^2 .* tanh(k12 * (t-td_LO)) * k12 - g;
figure(1)
hold on;
grid on;
plot(t,lref);
plot(t,dlref);
% plot(t,ddlref);

%% Flight
l_LO = k11 .* tanh(0) - 0.5*g*td_LO.^2 + k13*td_LO + k14;
dl_LO = k11 * k12 .* sech(0).^2 - g*td_LO + k13;
zCOMmax = l_LO + dl_LO^2/(2*g);
if zd_Fmax < 0 
    zd_Fmax = 0.1;
elseif zd_Fmax < zCOMmax - 0.71
    zd_Fmax = zCOMmax - 0.71 + 0.05;
elseif zd_Fmax > zCOMmax - 0.3
    zd_Fmax  = zCOMmax - 0.3 - 0.05;
end
% zd_Fmax is minimum 0 or zCOMmax-0.71 and max zCOMmax-0.3

tCOMmax = dl_LO/g; % time after which COM reaches max height
tCOMfall = sqrt(2*(zCOMmax-0.4)/g); % time after which COM will fall down to 0.4, thats the maximum time, after which foot should touch ground

if tz_Fmax > (tCOMmax + tCOMfall)*0.5
    tz_Fmax = (tCOMmax + tCOMfall)*0.5;
end


[k21, k22, k23] = optFlight(tz_Fmax, zd_Fmax);

% k21 = 0.13;
% k22 = 25;
% k23 = 0.01;

t = 0:0.001:2*tz_Fmax;
zFref = k21 * 1 ./ (cosh(k22 * (t - tz_Fmax)).^2) - k23;
dzFref = -2*k21*k22.*sinh(k22*(t-tz_Fmax)).*cosh(k22*(t-tz_Fmax))*k22 + k23;
figure(2)
hold on;
grid on;
plot(t,zFref);
% plot(t,dzFref);
% plot(t,ddlref);

%% Landing
lF = l_LO + dl_LO*(2*tz_Fmax) - 0.5*g*(2*tz_Fmax)^2; % height of COM at landing instance
dlF = dl_LO - g*(2*tz_Fmax); % speed of COM at landing instance

[k31, k32, k33] = optLanding(tl_min,tl_end,lF,dlF);

t = 0:0.01:tl_end;
lref_LI = -k31.*1./(cosh(k32*(t - tl_min)).^2)-k33;
dlref_LI = 2*k31*k32.*tanh(k32*(t-tl_min)).*sech(k32*(t-tl_min)).^2;

figure(3)
hold on;
grid on;
plot(t,lref_LI);
plot(t,dlref_LI);

%% Rejump
ld_LO = k11 .* tanh(0) - 0.5*g*td_LO.^2 + k13*td_LO + k14;
treJump = 1;


[k41, k42, k43, k44] = optReJump(treJump,ld_LO, dld_LO, lF, dlF);

t = 0:0.01:treJump;
lreJump = k41 .* tanh(k42 * (t - treJump)) - 0.5*g*t.^2 + k43*t + k44;
dlreJump = k41 * k42 .* sech(k42*(t-treJump)).^2 - g*t + k43;
figure(4)
hold on;
grid on;
plot(t,lreJump);
plot(t,dlreJump);
end




