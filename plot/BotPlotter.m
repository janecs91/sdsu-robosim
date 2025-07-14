classdef BotPlotter
    properties
    end
    methods(Static)
        function plot_triangle(obj, state, newState, x_a, y_a, dx_b, dy_b, x_c, y_c, turnAngle, altLeg)
            %% plot
            localJointPositions = obj.get_local_joint_positions(state);
            globalJointPositions = obj.get_global_joint_positions(state);
            newGlobalJointPositions = obj.get_global_joint_positions(newState);
            rotatedBodyVector = obj.rotateVector([dx_b dy_b], turnAngle);
            %{
            futureState = copy(state);
            futureState.baseOrientation = state.baseOrientation + turnAngle;
            futureGlobalJointPositions = obj.get_global_joint_positions(futureState);
            %}
            newLocalJointPositions = cell(4,1);
            for i=1:4
                jointPositionsDifference = newGlobalJointPositions{i}-globalJointPositions{i};
                newLocalJointPositions{i} = localJointPositions{i} + jointPositionsDifference;
            end
            x_plot = [x_a localJointPositions{3}(end,1) localJointPositions{4}(end,1) x_a];
            y_plot = [y_a localJointPositions{3}(end,2) localJointPositions{4}(end,2) y_a];
            x_plot2 = [x_a newLocalJointPositions{3}(end,1)+rotatedBodyVector(1) ...
                newLocalJointPositions{4}(end,1)+rotatedBodyVector(1) x_a];
            y_plot2 = [y_a newLocalJointPositions{3}(end,2)+rotatedBodyVector(2) ...
                newLocalJointPositions{4}(end,2)+rotatedBodyVector(2) y_a];
            fprintf("\n\nXXXXXXXXXXXX Plotting============\n")
            fprintf("%.2f %.2f %.2f %.2f %.2f %.2f\n", x_a, y_a, ...
                localJointPositions{3}(end,1:2), localJointPositions{4}(end,1:2));
            fprintf("%.2f %.2f\n\n", dx_b, dy_b);
            figure
            hold on
            grid on
            plot(x_plot, y_plot, 'LineWidth',2);
            plot(x_plot2, y_plot2, 'LineWidth',2);
            plot([0 rotatedBodyVector(1)], [0 rotatedBodyVector(2)], '--o', 'LineWidth',2);
            plot([x_plot(2) x_plot2(2)], [y_plot(2) y_plot2(2)], '--', 'LineWidth',2);
            plot([x_plot(3) x_plot2(3)], [y_plot(3) y_plot2(3)], '--', 'LineWidth',2);
            title(sprintf("leg %d", altLeg));
            xlabel('x');
            ylabel('y');
            
            scatter(dx_b, dy_b, 60, '+');
            scatter(x_c, y_c, 60, '*');
            scatter(0,0,60,'*');
            scatter(localJointPositions{altLeg}(:,1), localJointPositions{altLeg}(:,2), 60, 'd', 'filled');
            futureJointsX = newLocalJointPositions{altLeg}(:,1);
            futureJointsY = newLocalJointPositions{altLeg}(:,2)+rotatedBodyVector(2);
            scatter(futureJointsX, futureJointsY, 60, 'd', 'filled');
            hold off
            % text
            x_text = [x_plot x_plot2];
            y_text = [y_plot y_plot2];
            offset = [0.5 2];
            for t = 1:size(x_text,2)
              text(x_text(t)+offset(1), y_text(t)+offset(2),sprintf("%.2f, %.2f", x_text(t), y_text(t)));
            end
            text(0, 2, "(0,0)");
            text(rotatedBodyVector(1), rotatedBodyVector(2), sprintf("dx_b,dy_b(%.2f, %.2f)", ...
                rotatedBodyVector(1), rotatedBodyVector(2)));
            text(x_c, y_c, sprintf("tri ctr(%.2f, %.2f)", x_c, y_c));
            text(localJointPositions{altLeg}(:,1), localJointPositions{altLeg}(:,2), ["b1", "w1", "h1", "k1", "a1"]);        
            text(futureJointsX, futureJointsY, ["b2", "w2", "h2", "k2", "a2"]);
        end
    end
end