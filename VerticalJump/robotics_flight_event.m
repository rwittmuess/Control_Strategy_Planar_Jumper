function [value, isterminal, direction] = robotics_flight_event(~,s,~,~,~)
    value      = s(4); % foot back on ground: zF = 0
    isterminal = 1;  % Stop the integration
    direction  = -1; % (+1: detect zeros where the event funtion is increasing)
end