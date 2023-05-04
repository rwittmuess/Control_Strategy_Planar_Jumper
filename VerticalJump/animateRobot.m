% Animate the Three-Link Walker
function animateRobot(tData, qData)
    figure(1000)

    for i =1:length(tData) 
        clf ;
        drawRobot(qData(i, :)');
        line([-1, 1],[0;0],'Color', 'k', 'LineWidth', 2)
        axis([-1 1 -0.5 1.5]) ; 
        axis equal
        grid on ;
        drawnow ;
        pause(0.0001) ;
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
    l1 = line([xF;pKnee(1)], [zF;pKnee(2)], 'Color', 'k', 'LineWidth', 2);
    hold on
    l2 = line([pKnee(1);pHip(1)], [pKnee(2);pHip(2)], 'Color', 'k', 'LineWidth', 2);
    l3 = line([pHip(1);pHead(1)], [pHip(2);pHead(2)], 'Color', 'k', 'LineWidth', 2);
    plot(pKnee(1), pKnee(2), 'bo', 'MarkerSize',7,'MarkerEdgeColor','b','MarkerFaceColor','g')
    plot(pHip(1), pHip(2), 'ro', 'MarkerSize',7,'MarkerEdgeColor','r','MarkerFaceColor','g')
    plot(xF, zF, 'ko', 'MarkerSize',7,'MarkerEdgeColor','k','MarkerFaceColor','g')
    plot(pCOM(1), pCOM(2), 'ko', 'MarkerSize',4,'MarkerEdgeColor','k','MarkerFaceColor','r')
end