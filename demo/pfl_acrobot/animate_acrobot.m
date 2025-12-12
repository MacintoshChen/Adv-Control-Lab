% function animate_acrobot(t, x, params)
%     % Extract link lengths
%     l1 = params.l1;
%     l2 = params.l2;
% 
%     % Set up figure
%     figure; clf;
%     axis equal; grid on; hold on;
%     xlabel('X'); ylabel('Y');
%     title('Acrobot Animation');
%     axis([- (l1+l2),  (l1+l2),    - (l1+l2),   (l1+l2)]);
% 
%     % Plot objects
%     link1 = plot([0,0],[0,0], 'LineWidth', 3);
%     link2 = plot([0,0],[0,0], 'LineWidth', 3);
%     joint1 = plot(0,0,'ko','MarkerSize',8,'MarkerFaceColor','k');
%     joint2 = plot(0,0,'ko','MarkerSize',8,'MarkerFaceColor','k');
% 
%     for k = 1:length(t)
%         q1 = x(k,1);
%         q2 = x(k,2);
% 
%         % Forward kinematics
%         x1 = l1 * cos(q1);
%         y1 = l1 * sin(q1);
% 
%         x2 = x1 + l2 * cos(q1 + q2);
%         y2 = y1 + l2 * sin(q1 + q2);
%         % Update link plots
%         set(link1, 'XData', [0 x1], 'YData', [0 y1]);
%         set(link2, 'XData', [x1 x2], 'YData', [y1 y2]);
% 
%         % Update joint markers
%         set(joint1, 'XData', x1, 'YData', y1);
%         set(joint2, 'XData', x2, 'YData', y2);
% 
%         drawnow;
%         if k > 1
%             dt = t(k) - t(k-1);
%             pause(dt*0.01);   % real-time animation
%         end
%     end
% end
% 
function animate_acrobot(t, x, params)
    % Extract link lengths
    l1 = params.l1;
    l2 = params.l2;

    % Set up figure
    figure; 
    axis equal; grid on; hold on;
    xlabel('X'); ylabel('Y');
    title('Acrobot Animation with LQR Control');
    axis([- (l1+l2),  (l1+l2),    - (l1+l2),   (l1+l2)]);

    q1_des = params.q1_des;
    q2_des = params.q2_des;
    x1_des = l1 * cos(q1_des);
    y1_des = l1 * sin(q1_des);
    x2_des = x1_des + l2 * cos(q1_des + q2_des);
    y2_des = y1_des + l2 * sin(q1_des + q2_des);

    % Plot target position (transparent)
    plot([0, x1_des, x2_des], [0, y1_des, y2_des], 'r--', 'LineWidth', 1);
    plot(x2_des, y2_des, 'r*', 'MarkerSize', 10, 'LineWidth', 2);

    % Plot objects
    link1 = plot([0,0],[0,0], 'b-', 'LineWidth', 3);
    link2 = plot([0,0],[0,0], 'g-', 'LineWidth', 3);
    joint1 = plot(0,0,'ko','MarkerSize',8,'MarkerFaceColor','k');
    joint2 = plot(0,0,'ko','MarkerSize',8,'MarkerFaceColor','k');
    end_effector = plot(0,0,'bo','MarkerSize',6,'MarkerFaceColor','b');

    legend('Target Position', 'Target End-effector', 'Link1', 'Link2', ...
           'Joint1', 'Joint2', 'End-effector', 'Location', 'best');

    for k = 1:length(t)
        q1 = x(k,1);
        q2 = x(k,2);

        % Forward kinematics
        x1 = l1 * cos(q1);
        y1 = l1 * sin(q1);
        
        x2 = x1 + l2 * cos(q1 + q2);
        y2 = y1 + l2 * sin(q1 + q2);
        
        % Update link plots
        set(link1, 'XData', [0 x1], 'YData', [0 y1]);
        set(link2, 'XData', [x1 x2], 'YData', [y1 y2]);

        % Update joint markers
        set(joint1, 'XData', x1, 'YData', y1);
        set(joint2, 'XData', x2, 'YData', y2);
        set(end_effector, 'XData', x2, 'YData', y2);

        % time
        title(sprintf('Acrobot Animation with LQR Control (t=%.2f s)', t(k)));
        
        drawnow limitrate;
        if k > 1
            dt = t(k) - t(k-1);
            pause(dt*0.0001); 
        end
    end
end