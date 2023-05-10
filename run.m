%% Legged Robots - Final Project
%{
    Project finished: 2023 | 05 | 09 
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
zd_Fmax = 0.4; % max foot hight in the air (min max depends on jump-off, see below)
tz_Fmax = 0.2; % time after which max foot hight is reached, half of flight time

% Landing settings
tl_min = 0.2; % time after which lowest COM is reached
tl_end = 1; % time after which landing is finished (at as big as tl_min


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
plot(tData_GroundPhase,l, '--','LineWidth', 1.5); 
xlabel({'$t$ in [$s$]'}, 'Interpreter', 'latex') 
ylabel({'$l$ in [$m$]'}, 'Interpreter', 'latex') 
legend({'$l_{ref}$', '$l_{real}$'}, 'Interpreter', 'latex')
title({'Ground Phase: $l_{ref}$ and $l_{real}$'}, 'Interpreter', 'latex')
axis([tData_GroundPhase(1) tData_GroundPhase(end) 0 0.8])

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
plot(tData_FlightPhase,zFref, 'LineWidth', 1.5);hold on; grid on;
plot(tData_FlightPhase,zFreal,'--', 'LineWidth', 1.5);
xlabel({'$t$ in [$s$]'}, 'Interpreter', 'latex') 
ylabel({'$z_{ref}^{Foot}$ and $z_{real}^{Foot}$ in [$m$]'}, 'Interpreter', 'latex') 
legend({'$z_{ref}^{Foot}$','$z_{real}^{Foot}$'}, 'Interpreter', 'latex')
title({'Flight Phase: $z_{ref}^{Foot}$ and $z_{real}^{Foot}$'}, 'Interpreter', 'latex')

temp = gca;
exportgraphics(temp,'./plots/FlightPhase_zFref_zFreal.pdf','ContentType','vector')


%% LANDING PHASE
% lref vs lreal

l = l_gen(sData_LandingPhase(:,1:5)')';
tLI = tData_FlightPhase(end); 
lref_LI = lref_LI_gen([tData_LandingPhase';ones(1,length(tData_LandingPhase))*tLI;ones(1,length(tData_LandingPhase))*tl_min;]);


figure
hold on;
grid on;
plot(tData_LandingPhase,lref_LI, 'LineWidth', 1.5);hold on; grid on;
plot(tData_LandingPhase,l,'--', 'LineWidth', 1.5);
xlabel({'$t$ in [$s$]'}, 'Interpreter', 'latex') 
ylabel({'$l_{ref}$ and $l_{real}$ in [$m$]'}, 'Interpreter', 'latex') 
%end_value_x = floor(tData(end)) + ceil((tData(end)-floor(tData(end)))/0.25) * 0.25;
%end_value_y = floor(max(zCOM)) + ceil((max(zCOM)-floor(max(zCOM)))/0.25) * 0.25;
axis([tData_LandingPhase(1) tData_LandingPhase(end) 0 0.8])
legend({'$l_{ref}$','$l_{real}$'}, 'Interpreter', 'latex')
title({'Landing Phase: $l_{ref}$ and $l_{real}$'}, 'Interpreter', 'latex')

temp = gca;
exportgraphics(temp,'./plots/LandingPhase_lref_lreal.pdf','ContentType','vector')


%% Whole Jump
% zCOM & zFoot

zCOM = zCOM_gen(sData');

figure
hold on;
grid on;
plot(tData,zCOM, 'LineWidth', 1.5); 
plot(tData,sData(:,4), 'LineWidth', 1.5);
end_value_x = floor(tData(end)) + ceil((tData(end)-floor(tData(end)))/0.25) * 0.25;
end_value_y = floor(max(zCOM)) + ceil((max(zCOM)-floor(max(zCOM)))/0.25) * 0.25;
axis([0 end_value_x 0 end_value_y])

% highlight the area between beginning and end of the flight phase
area([tData_FlightPhase(1) tData_FlightPhase(end)], [end_value_y+.1 end_value_y+.1], 'FaceColor', [255/255,205/255,0/255], 'FaceAlpha', 0.4)

xlabel({'$t$ in [$s$]'}, 'Interpreter', 'latex') 
ylabel({'$z^{CoM}$ and $z^{Foot}$ in [$m$]'}, 'Interpreter', 'latex') 
legend({'$z^{CoM}$','$z^{Foot}$','Flight Phase'}, 'Interpreter', 'latex')
title({'Entire Jump Process $z^{CoM}$ and $z^{Foot}$'}, 'Interpreter', 'latex')

temp = gca;
exportgraphics(temp,'./plots/EntireJump_zCoM_zFoot.pdf','ContentType','vector')

