function [value, isterminal, direction] = robotics_event(t,s,td_LO)
    value      = t - td_LO;
    isterminal = 1;  % Stop the integration
    direction  = +1; % (+1: detect zeros where the event funtion is increasing)
end