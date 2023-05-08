function [value, isterminal, direction] = robotics_event(t,s,td_LO)
    value      = t - td_LO;
    isterminal = 1;  % Stop the integration
    direction  = +1; % (+1: detect zeros where the event funtion is increasing)
end



% function [value, isterminal, direction] = robotics_event(t,s)
% 
%     persistent dl_prev % Declare ddl_prev as a persistent variable
% 
%     if isempty(dl_prev) % For the first integration step, use ddl = 0
%         dl = dl_gen(s);
%         ddl = 0;
%     else
%         dl = dl_gen(s);
%         ddl = (dl-dl_prev)/0.01;
%     end
%     dl_prev = dl;
% 
% 
%     if t-0.7<0
%         value = (t-1);
%     else
%         disp([num2str(dl_prev), ' | ',num2str(t)])
%         % disp('??????????????????????????????????')
%         value = (t-1)+(-ddl+9.81)-15; % SOMETHING WRONG HERE
%         % ddl is making problems
%     end
% 
% 
%     isterminal = 1;  % Stop the integration
%     direction  = +1; % (+1: detect zeros where the event funtion is increasing)
% 
%     % disp(['value: ', num2str(value), ' | t: ', num2str(t)])
% end

% if t<0.3
%     value = ddl+9.81-t-10;
%     disp(['t small | t: ', num2str(t), ' | value: ', num2str(value)])
% else
%     value = ddl+9.81;
%     disp(['t big', num2str(value)])
% end

% function [value, isterminal, direction] = robotics_event(t,s)
% 
%     % persistent dl_prev % Declare ddl_prev as a persistent variable
%     % 
%     % disp("Start")
%     % 
%     % if isempty(dl_prev) % For the first integration step, use ddl = 0
%     %     dl = dl_gen(s);
%     %     ddl = 0;
%     %     disp("Empty")
%     % else
%     %     dl = dl_gen(s);
%     %     ddl = dl-dl_prev*0.01;
%     % end
% 
%     % disp(ddl)
% 
%     value = t-0.5; %-0.5+t - 9.81+ddl
% 
%     disp(value)
% 
%     % if (ddl<(-9.81)) && (t-0.3>0) % && -dxCOM_gen(s)+1 == 0
%     %     disp("QUIT!")
%     %     value = 0;
%     % else
%     %     value = 1;
%     % end
%     % if dzCOM_gen(s)-0.5 == 0 && -dxCOM_gen(s)+1 == 0
%     %     value = 0;
%     % else
%     %     value = 1;
%     % end
%     isterminal = 1;  % Stop the integration
%     direction  = 0; % (+1: detect zeros where the event funtion is increasing)
% 
%     % dl_prev = dl;
% 
% end
% 
% % function [value, isterminal, direction] = robotics_event(t,~)
% %     td_LO = 1;
% %     value      = t - td_LO;
% %     isterminal = 1;  % Stop the integration
% %     direction  = +1; % (+1: detect zeros where the event funtion is increasing)
% % end