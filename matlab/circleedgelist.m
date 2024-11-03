%CIRCLEEDGELIST Return list of circle pixels

function [c] = circleedgelist(P1, P2, radius)
    
    % "radius" is the redius of the circle

    % Given two neighboring points and the radius
    x1 = P1(1); % start point
    y1 = P1(2);
    x2 = P2(1); % neighbor
    y2 = P2(2);

    % Calculate the midpoint between the two points
    center_x = (x1 + x2) / 2;
    center_y = (y1 + y2) / 2;

    % Calculate the direction vector of the line connecting the two points
    dx = x2 - x1;
    dy = y2 - y1;

    % Calculate the length of the direction vector
    length = sqrt(dx^2 + dy^2);

    % Normalize the direction vector
    dx = dx / length;
    dy = dy / length;

    % Calculate the coordinates of the center of the circle
    center_x = center_x + radius * dy;
    center_y = center_y - radius * dx; % Note the minus sign here

    % Create an array of angles to represent the circle
    angles = linspace(0, 2*pi, 100); % You can adjust the number of points for a smoother circle

    % Calculate the coordinates of points on the circle using the parametric equations
    x_circle = (round(center_x + radius * cos(angles)));
    y_circle = (round(center_y + radius * sin(angles)));

    c = [x_circle ; y_circle];

    i = find(c(1,:) == P2(1) & c(2,:) == P2(2), 1, 'first');

    temp = c(:,1:i-1);
    c(:,1:i-1) = [];
    c = [c temp];

    c = unique(c','rows','stable')';

% figure;
%     % Plot the circle and the two neighboring points
%     plot(c(1,:), c(2,:), 'b', 'LineWidth', 2);
%     hold on;
%     plot([x1, x2], [y1, y2], 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
%     axis equal;
%     xlabel('x');
%     ylabel('y');
%     title('Circle with Specified Radius passing through Two Neighboring Points');
%     grid on;
%     hold off;



end






