%% Legged Robots - Final Project
% To-Do:
% make event function dependend on l'' < -g for lift-off
% plot desired path and actual path to make sure robot follows desired path
% change robotics_flight to reduced controler!!!



%% For a fresh start:
clc; clear; close all;
dynamics;

%% Settings for increasing computational speed

reltol_start    = 1e-2;
abstol_start    = 1e-3;
tspan_start     = 0:0.01:1;   % tspan_start = [0 1];

reltol_flight   = 1e-2;
abstol_flight   = 1e-3;
% tspan_flight    = [0:0.01:1];   % tspan_start = [0 1];


%% Solve ODE for robot dynamics

%s0 = [0;-pi/4;pi/2;0;0;...
      % 0;0;0;0;0];
s0 = [0;-pi/4;pi/2;0;0;...
      0;0;0;0;0];

Opt = odeset('Events', @robotics_event,'RelTol',reltol_start,'AbsTol',abstol_start);
[t,s] = ode45(@(t,s) robot_dynamics(t,s), tspan_start, s0, Opt);

tData = t(1:end-1);
sData = s(1:end-1,:);

%% after running once: just run from here
% s0 = [0.643551279512161, -1.38116878810655, 0.737613340437614, -4.36564827749447e-15, 2.25977590679604e-14, -1.04637683867421, 2.32542793026359, -1.28181713291239, -6.02979306700862e-15, 3.01614185027691e-14];

s0 = sData(end,:);

LOv = [l_gen(sData(end,1:5)');dl_gen(sData(end,:)')];

Opt = odeset('Events', @robotics_flight_event,'RelTol',reltol_flight,'AbsTol',abstol_flight);
% [t,s] = ode45(@(t,s) robot_dynamics_flight(t,s,LOv), tData(end):0.001:tData(end)+10, s0, Opt); %[0:0.005:0.15]
[t,s] = ode45(@(t,s) robot_dynamics_flight(t,s,LOv), tData(end):0.01:tData(end)+3, s0, Opt); %[0:0.005:0.15]


% tData = [tData;tData(end)+t(2:end)];
tData = [tData;t(2:end)];
sData = [sData;s(2:end,:)];

%% prepare next jump
s0 = sData(end,:);

Opt = odeset('Events', @robotics_event,'RelTol',reltol_start,'AbsTol',abstol_start);
[t,s] = ode45(@(t,s) robot_dynamics(t,s), tData(end):0.01:tData(end)+3, s0, Opt);

tData = [tData;t(2:end)];
sData = [sData;s(2:end,:)];


%%
% t = 0;
% q = [0,-pi/4,pi/2,0,0];

qData = sData(:,1:5);

animateRobot(tData,qData)

%%
lref_LO = lref_LO_gen(tData);
l = l_gen(qData')';
% t1s_val = [tData';ones(length(tData)*1)];
% lref_F = lref_F_gen(t1s_val,sData);
% lref_F = [zeros(1,101),lref_F]';
% zFref = zFref_gen(t1s_val);

%%

% ds = robot_dynamics(tData,sData');

figure(1)
plot(tData,l);
hold on;
grid on;
plot(tData,lref_LO);
legend(["l_{real}", "lref_{LO}"]);

dlref_LO = dlref_LO_gen(tData);
dl = dl_gen(sData')';

figure(2)
plot(tData,dl);
hold on;
grid on;
plot(tData,dlref_LO);
legend(["dl_{real}", "dlref_{LO}"])
% 
% ds2 = [sData, ds(6:end,:)'];
% 
% ddl = ddl_gen(ds2')';
% 
% ddl = diff(dl)/(tData(2)-tData(1));

%%
tF = tData(101:end);
t1s_val = [tF';ones(1,length(tF))*1];
%%
zFref = zFref_gen(t1s_val);
figure(3)
hold on;
grid on;
plot(tF,zFref);
plot(tF,sData(101:end,4));
legend(["zF_{ref}","zF_{real}"]);

%%
% lref_F = lref_F_gen([tF';ones(1,length(tF))*1;ones(1,length(tF))*LOv(1);ones(1,length(tF))*LOv(2)]);
zCOM_kin = zCOM_kin_gen([tF';ones(1,length(tF))*1;ones(1,length(tF))*LOv(1);ones(1,length(tF))*LOv(2)]);
zFref = zFref_gen([tF';ones(1,length(tF))*1]);
pCOM = pCOM_gen(sData(101:end,1:5)');
zCOM = pCOM(2,:);

lref_F = zCOM - zFref;

figure(4)
hold on;
grid on;
plot(tF,lref_F);
plot(tF,zCOM_kin);
plot(tF,zFref);
% plot(tF,sData(102:end,4));
plot(tF,zCOM);
plot(tF,l(101:end));

legend(["lref_F", "zCOM_{kin}", "zF_{ref}", "zCOM_{real}","l_{real}"]);
% plot(tF,sData(102:end,4));


dzCOM_kin = dzCOM_kin_gen([tF';ones(1,length(tF))*1;ones(1,length(tF))*LOv(1);ones(1,length(tF))*LOv(2)]);
dzCOM = dzCOM_gen(sData(101:end,:)');



figure(5)
hold on;
grid on;
plot(tF,dzCOM_kin);
plot(tF,dzCOM);
legend(["dzCOM_{kin}", "dzCOM_{real}"])

tTest = 1:0.01:1.5;
zFref = zFref_gen([tTest;ones(1,length(tTest))*1]);

figure(6)
hold on;
grid on;
plot(tTest,zFref);
