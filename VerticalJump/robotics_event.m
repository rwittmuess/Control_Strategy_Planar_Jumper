function [value, isterminal, direction] = robotics_event(t,s)
    if dzCOM_gen(s)-0.5 == 0 && -dxCOM_gen(s)+1 == 0
        value = 0;
    else
        value = 1;
    end
    isterminal = 1;  % Stop the integration
    direction  = +1; % (+1: detect zeros where the event funtion is increasing)
end

% function [value, isterminal, direction] = robotics_event(t,~)
%     td_LO = 1;
%     value      = t - td_LO;
%     isterminal = 1;  % Stop the integration
%     direction  = +1; % (+1: detect zeros where the event funtion is increasing)
% end