% Animate the Vertical Jumper
function animateRobot(tData, qData)
    figure
    for i = 1:length(tData)
        clf;
        drawFloor;
        drawRobot(qData(i,:)');
        axis equal;
        axis([-1 1 -0.5 1.5]);
        grid on;
        drawnow;
        % pause(0.0001) ;
        % Make it dependent on the "real" time steps
        if i < length(tData)
            pause(tData(i+1) - tData(i));
        end
        % To save the animation as a GIF:
        % exportgraphics(gcf,'animation_jump.gif','Append',true);
    end
end

% Draw one frame of the Three-Link Walker
function drawRobot(q)
    % q1 = q(1);
    % q2 = q(2);
    % q3 = q(3);
    zF = q(4);
    xF = q(5);

    pKnee = pKnee_gen(q);
    pHip = pHip_gen(q);
    pHead = pHead_gen(q);
    pCOM = pCOM_gen(q);

    h = zeros(1,7);

    h(1) = line([xF;pKnee(1)], [zF;pKnee(2)], 'Color', 'k', 'LineWidth', 2,'DisplayName','lower leg');
        hold on;
    h(2) = line([pKnee(1);pHip(1)], [pKnee(2);pHip(2)], 'Color', 'k', 'LineWidth', 2,'DisplayName','upper leg');
    h(3) = line([pHip(1);pHead(1)], [pHip(2);pHead(2)], 'Color', 'k', 'LineWidth', 2,'DisplayName','body');
    h(4) = plot(pHip(1), pHip(2), 'ko', 'MarkerSize',7,'MarkerFaceColor',[217/255, 102/255, 31/255],'DisplayName','hip');
    h(5) = plot(pKnee(1), pKnee(2), 'ko', 'MarkerSize',7,'MarkerFaceColor',[59/255, 126/255, 161/255],'DisplayName','knee');
    h(6) = plot(xF, zF, 'ko', 'MarkerSize',7,'MarkerFaceColor',[253/255, 181/255, 21/255],'DisplayName','foot');
    h(7) = plot(pCOM(1), pCOM(2), 'ko', 'MarkerSize',4,'MarkerEdgeColor','k','MarkerFaceColor','r','DisplayName','CoM'); 
        hold off;
    legend(h(4:7));
end

function drawFloor
    % Define the number of stripes
    num_stripes = 10;
    
    % Define the width of each stripe
    stripe_width = 0.1;
    
    % Draw the diagonal lines
    for i = -num_stripes:num_stripes
        line([i*stripe_width, i*stripe_width-0.1], [0, -0.1], 'Color', 'k', 'LineWidth', 1);
    end
    
    % % Set the axis limits
    % xlim([0 num_stripes*stripe_width]);
    % ylim([-1 0]);
    % 
    % % Set the aspect ratio to equal
    % daspect([1 1 1]);
    line([-1, 1], [0;0], 'Color', 'k', 'LineWidth', 1);

end