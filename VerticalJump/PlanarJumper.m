function [  tData, sData, ...
            tData_GroundPhase, sData_GroundPhase, ...
            tData_FlightPhase, sData_FlightPhase, ...
            tData_LandingPhase, sData_LandingPhase,...
            LOv] = PlanarJumper(td_LO,tz_Fmax,tl_min,tl_end)
%% Settings for increasing computational speed
reltol_start        = 1e-2;
abstol_start        = 1e-3;
step_size_start     = 0.01;

reltol_flight       = 1e-2;
abstol_flight       = 1e-3;
step_size_flight    = 0.01;

reltol_landing      = 1e-2;
abstol_landing      = 1e-3;
step_size_landing   = 0.01;


%% GROUND PHASE
s0 = [0;-pi/4;pi/2;0;0;...
      0;0;0;0;0];

Opt = odeset('Events', @(t,s)robotics_event(t,s,td_LO),'RelTol',reltol_start,'AbsTol',abstol_start);
[t,s] = ode45(@(t,s) robot_dynamics_ground(t,s,td_LO), 0:step_size_start:td_LO, s0, Opt);

tData = t(1:end);
sData = s(1:end,:);
tData_GroundPhase = t;
sData_GroundPhase = s;


%% FLIGHT PHASE
% s0 = [0.643551279512161, -1.38116878810655, 0.737613340437614, -4.36564827749447e-15, 2.25977590679604e-14, -1.04637683867421, 2.32542793026359, -1.28181713291239, -6.02979306700862e-15, 3.01614185027691e-14];
s0 = sData(end,:);

LOv = [l_gen(sData(end,1:5)');dl_gen(sData(end,:)')];

Opt = odeset('Events', @robotics_flight_event,'RelTol',reltol_flight,'AbsTol',abstol_flight);
[t,s] = ode45(@(t,s) robot_dynamics_flight(t,s,LOv,td_LO,tz_Fmax), tData(end):step_size_flight:tData(end)+3, s0, Opt); %[0:0.005:0.15]

% tData = [tData;tData(end)+t(2:end)];
tData = [tData;t(2:end)];
sData = [sData;s(2:end,:)];
tData_FlightPhase = t;
sData_FlightPhase = s;


%% LANDING PHASE
s0 = sData(end,:);
% -> = [0.7122   -1.6219    0.9931   -0.0000   -0.0052    0.6455   -2.2303  2.2222   -0.5956   -0.0477]; we get this
%--> = [0.7122   -1.6219    0.9931   -0.0000   -0.0052    0.6455   -2.2303  2.2222   -0.0000   -0.0000]; we modify it to this
% s0 = [0.6437   -1.3817    0.7380   -0.0000   -0.0000   -1.2026    2.5749 -1.4090   -0.0000   -0.0000]; % s(end) from Ground_Phase

s0(9:10) = [0,0]; % to satisfy assumptions of the paper


t_LI = tData(end);

Opt = odeset('Events', @(t,s)robotics_landing_event(t,s,t_LI,tl_min,tl_end),'RelTol',reltol_landing,'AbsTol',abstol_landing);
[t,s] = ode45(@(t,s) robot_dynamics_landing(t,s,t_LI,tl_min,tl_end), tData(end):step_size_landing:tData(end)+2, s0, Opt);

tData = [tData;t(2:end)];
sData = [sData;s(2:end,:)];
tData_LandingPhase = t;
sData_LandingPhase = s;


end