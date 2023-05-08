function [value, isterminal, direction] = robotics_landing_event(t,~,t_LI,tl_min,tl_end)  
    value      = t - t_LI - tl_end; % stop after 1 second    
    isterminal = 1;  % Stop the integration
    direction  = +1; % (+1: detect zeros where the event funtion is increasing)
end