%% Legged Robots - Final Project
%{
    ToDo:
    * make event function dependend on l'' < -g for lift-off
    * plot desired path and actual path to make sure robot follows desired path
    * 
    * calculate K_i's for desired height/speed/time,...

    * take desired tz_min and other values into the dynamics to generate ts
    * and tss

%}


%% For a fresh start:
clc; clear; close all;


%% Setting Desired Values by User

% remember that the robots height is about 1m
% make sure that only plausible desired values are passed in and set them
% to max / min values if they are not plausible

% if robot is completely straight max COM is 0.71m
% if robot is squiched the minimal height of COM is 0.3m

% Jump-off settings
td_LO = 1; % time of lift-off (min = 0.1, max = 1)
dld_LO = 0.8; % speed of CoM at lift-off (max = 0.9, min = 0.1)

% Flight settings
zd_Fmax = 0.5; % max foot hight in the air (min max depends on jump-off, see below)
tz_Fmax = 0.2; % time after which max foot hight is reached, half of flight time

% Landing settings
tl_min = 0.2; % time after which lowest COM is reached
tl_end = 1; % time after which landing is finished (at as big as tl_min

% Re-jump settings
% t_rejump = 1; % time between landing and jump-off


%% Optimizing Values
[   k11, k12, k13, k14,...
    k21, k22, k23,...
    k31, k32, k33,...
    zd_Fmax, tz_Fmax] = optimizingReferences(td_LO, dld_LO, zd_Fmax, tz_Fmax, tl_min, tl_end);


%% Generate Dynamics
dynamics(   k11, k12, k13, k14,...
            k21, k22, k23,...
            k31, k32, k33);


%% Run Simulation
[   tData, sData, ...
    tData_GroundPhase, sData_GroundPhase, ...
    tData_FlightPhase, sData_FlightPhase, ...
    tData_LandingPhase, sData_LandingPhase,...
    LOv] = PlanarJumper(td_LO,tz_Fmax,tl_min,tl_end);


%% ANIMATION
qData = sData(:,1:5);
animateRobot(tData,qData)


%% GROUND PHASE: 
% l_ref & l_real
lref_LO = lref_LO_gen([tData_GroundPhase';ones(1,length(tData_GroundPhase))*td_LO]);
l = l_gen(sData_GroundPhase(:,1:5)')';

figure
hold on;
grid on;
plot(tData_GroundPhase,lref_LO, 'LineWidth', 1.5);hold on; grid on;
plot(tData_GroundPhase,l, 'LineWidth', 1.5); 
xlabel({'$t$ in [$s$]'}, 'Interpreter', 'latex') 
ylabel({'$l$ in [$m$]'}, 'Interpreter', 'latex') 
legend({'$l_{ref}$', '$l_{real}$'}, 'Interpreter', 'latex')
title({'Ground Phase: $l_{ref}$ and $l_{real}$'}, 'Interpreter', 'latex')

temp = gca;
exportgraphics(temp,'./plots/GroundPhase_lref_l.pdf','ContentType','vector')


% dl_ref & dl_real
dlref_LO = dlref_LO_gen([tData_GroundPhase';ones(1,length(tData_GroundPhase))*td_LO]);
dl = dl_gen(sData_GroundPhase')';

figure
plot(tData_GroundPhase,dlref_LO, 'LineWidth', 1.5); hold on; grid on;
plot(tData_GroundPhase,dl, 'LineWidth', 1.5); 
xlabel({'$t$ in [$s$]'}, 'Interpreter', 'latex') 
ylabel({'$\dot{l}$ in [$\frac{m}{s}$]'}, 'Interpreter', 'latex') 
legend({'$\dot{l}_{ref}$', '$\dot{l}_{real}$'}, 'Interpreter', 'latex')
title({'Ground Phase: $\dot{l}_{ref}$ and $\dot{l}_{real}$'}, 'Interpreter', 'latex')

temp = gca;
exportgraphics(temp,'plots/GroundPhase_dlref_dl.pdf','ContentType','vector')


%% FLIGHT PHASE
%z_F_ref & z_F_real

l_LO    = LOv(1);
dl_LO   = LOv(2);


zFref   = zFref_gen([tData_FlightPhase';ones(1,length(tData_FlightPhase))*td_LO;ones(1,length(tData_FlightPhase))*tz_Fmax]);
zFreal  = zCOM_kin_gen([tData_FlightPhase';ones(1,length(tData_FlightPhase))*td_LO;ones(1,length(tData_FlightPhase))*tz_Fmax;ones(1,length(tData_FlightPhase))*l_LO;ones(1,length(tData_FlightPhase))*dl_LO]) ...
        - l_gen(sData_FlightPhase(:,1:5)');

figure
hold on;
grid on;
plot(tData_FlightPhase,zFref, 'LineWidth', 3);hold on; grid on;
plot(tData_FlightPhase,zFreal, 'LineWidth', 1.5);
xlabel({'$t$ in [$s$]'}, 'Interpreter', 'latex') 
ylabel({'$z_{ref}^{Foot}$ and $z_{real}^{Foot}$ in [$m$]'}, 'Interpreter', 'latex') 
legend({'$z_{ref}^{Foot}$','$z_{real}^{Foot}$'}, 'Interpreter', 'latex')
title({'Flight Phase: $z_{ref}^{Foot}$ and $z_{real}^{Foot}$'}, 'Interpreter', 'latex')

temp = gca;
exportgraphics(temp,'./plots/FlightPhase_zFref_zFreal.pdf','ContentType','vector')


%%
%zCOM & zF

zCOM = zCOM_gen(sData');

figure
hold on;
grid on;
plot(tData,zCOM, 'LineWidth', 1.5); 
plot(tData,sData(:,4), 'LineWidth', 1.5);
end_value_x = floor(tData(end)) + ceil((tData(end)-floor(tData(end)))/0.25) * 0.25;
end_value_y = floor(max(zCOM)) + ceil((max(zCOM)-floor(max(zCOM)))/0.25) * 0.25;
axis([0 end_value 0 end_value_y])

% highlight the area between beginning and end of the flight phase
area([tData_FlightPhase(1) tData_FlightPhase(end)], [end_value_y+.1 end_value_y+.1], 'FaceColor', 'green', 'FaceAlpha', 0.4)

xlabel({'$t$ in [$s$]'}, 'Interpreter', 'latex') 
ylabel({'$z^{CoM}$ and $z^{Foot}$ in [$m$]'}, 'Interpreter', 'latex') 
legend({'$z^{CoM}$','$z^{Foot}$','Flight Phase'}, 'Interpreter', 'latex')
title({'Entire Jump Process $z^{CoM}$ and $z^{Foot}$'}, 'Interpreter', 'latex')

temp = gca;
exportgraphics(temp,'./plots/EntireJump_zCoM_zFoot.pdf','ContentType','vector')


%%
% t1s_val = [tData';ones(length(tData)*1)];
% lref_F = lref_F_gen(t1s_val,sData);
% lref_F = [zeros(1,101),lref_F]';
% zFref = zFref_gen(t1s_val);
%% 
% 
% ds2 = [sData, ds(6:end,:)'];
% 
% ddl = ddl_gen(ds2')';
% 
% ddl = diff(dl)/(tData(2)-tData(1));

%%
% tF = tData(101:end);
% t1s_val = [tF';ones(1,length(tF))*1];
% %%
% zFref = zFref_gen(t1s_val);
% figure(3)
% hold on;
% grid on;
% plot(tF,zFref);
% plot(tF,sData(101:end,4));
% legend(["zF_{ref}","zF_{real}"]);
% 
% %%
% % lref_F = lref_F_gen([tF';ones(1,length(tF))*1;ones(1,length(tF))*LOv(1);ones(1,length(tF))*LOv(2)]);
% zCOM_kin = zCOM_kin_gen([tF';ones(1,length(tF))*1;ones(1,length(tF))*LOv(1);ones(1,length(tF))*LOv(2)]);
% zFref = zFref_gen([tF';ones(1,length(tF))*1]);
% pCOM = pCOM_gen(sData(101:end,1:5)');
% zCOM = pCOM(2,:);
% 
% lref_F = zCOM - zFref;
% 
% figure(4)
% hold on;
% grid on;
% plot(tF,lref_F);
% plot(tF,zCOM_kin);
% plot(tF,zFref);
% % plot(tF,sData(102:end,4));
% plot(tF,zCOM);
% plot(tF,l(101:end));
% 
% legend(["lref_F", "zCOM_{kin}", "zF_{ref}", "zCOM_{real}","l_{real}"]);
% % plot(tF,sData(102:end,4));
% 
% 
% dzCOM_kin = dzCOM_kin_gen([tF';ones(1,length(tF))*1;ones(1,length(tF))*LOv(1);ones(1,length(tF))*LOv(2)]);
% dzCOM = dzCOM_gen(sData(101:end,:)');
% 
% 
% figure(5)
% hold on;
% grid on;
% plot(tF,dzCOM_kin);
% plot(tF,dzCOM);
% legend(["dzCOM_{kin}", "dzCOM_{real}"])
% 
% tTest = 1:0.01:1.5;
% zFref = zFref_gen([tTest;ones(1,length(tTest))*1]);
% 
% figure(6)
% hold on;
% grid on;
% plot(tTest,zFref);