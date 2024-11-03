%Shortcut_Bug2 Bug
%
% Methods::
%   Bug2        Constructor
%   query       Find a path from start to goal
%   plot        Display the obstacle map
%   display     Display state/parameters in human readable form
%   char        Convert to string
%

classdef Shortcut_Bug2 < Navigation_ColorChanged

    properties(Access=protected)
        H           % hit points
        j           % number of hit points
        mline       % line from starting position to goal
        Temp_mline  % line from current position to m-line
        mPoint      % closest point to current position on m-line
        step        % state, in step 1 or step 2 of algorithm
        edge        % edge list
        k           % edge index
    end

    methods

        function bug = Shortcut_Bug2(varargin)
          
            % invoke the superclass constructor
            bug = bug@Navigation_ColorChanged(varargin{:});

            bug.H = [];
            bug.j = 1;
            bug.step = 1;
        end

        function pp = query(bug, start, goal, varargin)
       
            disp('Shortcut_Bug2 is running ... ')
         
            opt.animate = false;
            opt.movie = [];
            opt.current = false;
            
            opt = tb_optparse(opt, varargin);
            
            if ~isempty(opt.movie)
                anim = Animate(opt.movie);
                opt.animate = true;
            end
       
            % make sure start and goal are set and valid
            bug.start = []; bug.goal = [];
            bug.checkquery(start, goal);
            bug.mPoint = bug.start';
            
            % compute the m-line
            %  create homogeneous representation of the line
            %  line*[x y 1]' = 0
            bug.mline = homline(bug.start(1), bug.start(2), ...
                bug.goal(1), bug.goal(2));
            bug.mline = bug.mline / norm(bug.mline(1:2));
            
            if opt.animate
                bug.plot();
                
                bug.plot_mline();
            end
            
            % iterate using the next() method until we reach the goal
            robot = bug.start(:);
            bug.step = 1;
            path = bug.start(:);
            while true
                if opt.animate
                    plot(robot(1), robot(2), 'g.', 'MarkerSize', 12);
                    if opt.current
                        h = plot(robot(1), robot(2), 'ko', 'MarkerSize', 8);
                    end
                    drawnow
                    if ~isempty(opt.movie)
                        anim.add();
                    end
                    if opt.current
                        delete(h)
                    end
                end

                % move to next point on path
                robot = bug.next(robot);

                % are we there yet?
                if isempty(robot)
                    % yes, exit the loop
                    break
                else
                    % no, append it to the path
                    path = [path robot(:)];
                end
            end
            
            if ~isempty(opt.movie)
                anim.close();
            end

            % only return the path if required
            if nargout > 0
                pp = path';
            end
        end
        
        function plot_mline(bug, ls)
            
                % parameters of the M-line, direct from initial position to goal
                % as a vector mline, such that [robot 1]*mline = 0
                
                if nargin < 2
                    ls = 'k--';
                end
                dims = axis;
                xmin = dims(1); xmax = dims(2);
                ymin = dims(3); ymax = dims(4);
                
                hold on
                if bug.mline(2) == 0
                    % handle the case that the line is vertical
                    plot([start(1) start(1)], [ymin ymax], 'k--');
                else
                    x = [xmin xmax]';
                    y = -[x [1;1]] * [bug.mline(1); bug.mline(3)] / bug.mline(2);
                    plot(x, y, ls);
                end
        end

        function plot_Temp_mline(bug, ls)
            
                % parameters of the M-line, direct from initial position to goal
                % as a vector mline, such that [robot 1]*mline = 0
                
                if nargin < 2
                    ls = 'k--';
                end
                dims = axis;
                xmin = dims(1); xmax = dims(2);
                ymin = dims(3); ymax = dims(4);
                
                hold on
                if bug.Temp_mline(2) == 0
                    % handle the case that the line is vertical
                    plot([start(1) start(1)], [ymin ymax], 'k--');
                else
                    x = [xmin xmax]';
                    y = -[x [1;1]] * [bug.Temp_mline(1); bug.Temp_mline(3)] / bug.Temp_mline(2);
                    plot(x, y, ls);
                end
        end
        
        function n = next(bug, robot)
            
            % implement the main state machine for bug2
            n = [];
            robot = robot(:);
            % these are coordinates (x,y)
          
            if bug.step == 1
                % Step 1.  Move along the M-line toward the goal
                if colnorm(bug.goal - robot) == 0 % are we there yet?
                    return
                end

                % motion on line toward goal
                d = bug.goal-robot;
                if abs(d(1)) > abs(d(2))
                    % line slope less than 45 deg
                    dx = sign(d(1));
                    L = bug.mline;
                    y = -( (robot(1)+dx)*L(1) + L(3) ) / L(2);
                    dy = round(y - robot(2));
                else
                    % line slope greater than 45 deg
                    dy = sign(d(2));
                    L = bug.mline;
                    x = -( (robot(2)+dy)*L(2) + L(3) ) / L(1);
                    dx = round(x - robot(1));
                end
                

                % detect if next step is an obstacle
                if bug.isoccupied(robot + [dx; dy])
                    bug.message('(%d,%d) obstacle!', n);
                    bug.H(bug.j,:) = robot; % define hit point
                    bug.step = 2;
                    % get a list of all the points around the obstacle
                    bug.edge = edgelist(bug.occgridnav == 0, robot);
                    bug.k = 2;  % skip the first edge point, we are already there
                else
                    n = robot + [dx; dy];
                end
            end % step 1

            if bug.step == 2
                % Step 2.  Move around the obstacle until we reach a point
                % on the M-line closer than when we started.
                if colnorm(bug.goal-robot) == 0 % are we there yet?
                    return
                end

                if bug.k <= numcols(bug.edge)
                    n = bug.edge(:,bug.k);  % next edge point
                else
                    % we are at the end of the list of edge points, we
                    % are back where we started.  Step 2.c test.
                    error('RTB:bug2:noplan', 'robot is trapped')
                    return;
                end

                % are we on the M-line now ?
                if abs( [robot' 1]*bug.mline') <= 0.5
                    bug.message('(%d,%d) moving along the M-line', n);
                    % are closer than when we encountered the obstacle?
                    if colnorm(robot-bug.goal) < colnorm(bug.H(bug.j,:)'-bug.goal)
                        % back to moving along the M-line
                        bug.j = bug.j + 1;
                        bug.step = 1;
                        n = robot;
                        return;
                    end
                end

                % can we go to the M-line ?
                % Calculate the perpendicular distance of current position from the main m-line
                distance = abs(bug.mline(1)*robot(1) + bug.mline(2)*robot(2) + bug.mline(3)) / sqrt(bug.mline(1)^2 + bug.mline(2)^2);
                % Calculate the intersection point (closest point)
                intersection_x = robot(1) + bug.mline(1) * distance;
                intersection_y = robot(2) + bug.mline(2) * distance;

                % Is the new mPoint farther than previous mPoitn from start poinrt
                if colnorm([intersection_x intersection_y]' - bug.start) > colnorm(bug.mPoint' - bug.start)+0.5
                    bug.mPoint = [intersection_x intersection_y];
                    % if colnorm(bug.mPoint'-bug.goal) < colnorm(bug.H(bug.j,:)'-bug.goal)
                        bug.message('(%d,%d) moving toward the M-line', n);
                        % compute the temp m-line
                        bug.Temp_mline = homline(robot(1), robot(2), ...
                            bug.mPoint(1), bug.mPoint(2));
                        bug.Temp_mline = bug.Temp_mline / norm(bug.Temp_mline(1:2));

                        % check if the first point on the Temp_mline is occoppied
                        % motion on line toward mpoint
                        d = bug.mPoint'-robot;
                        if abs(d(1)) > abs(d(2))
                            % line slope less than 45 deg
                            dx = sign(d(1));
                            L = bug.Temp_mline;
                            y = -( (robot(1)+dx)*L(1) + L(3) ) / L(2);
                            dy = round(y - robot(2));
                        else
                            % line slope greater than 45 deg
                            dy = sign(d(2));
                            L = bug.Temp_mline;
                            x = -( (robot(2)+dy)*L(2) + L(3) ) / L(1);
                            dx = round(x - robot(1));
                        end
                        % disp(robot + [dx; dy])

                        % detect if next step is an obstacle
                        if bug.isoccupied(robot + [dx; dy])
                            % disp('occuppied')
                            bug.message('(%d,%d) obstacle!', n);
                        else
                            % disp('free')
                            bug.step = 3;
                            n = robot;
                            return;
                        end
                    % end
                end

                % no, keep going around
                bug.message('(%d,%d) keep moving around obstacle', n)
                bug.k = bug.k+1;
            end % step 2

            if bug.step == 3
                % disp(bug.step)
                % Step 3.  Move along the temp-mline toward the closest point

                if colnorm(bug.mPoint' - robot) == 0 % are we there yet?
                    bug.step = 1;
                    return
                end

                % motion on line toward goal
                d = bug.mPoint'-robot;
                if abs(d(1)) > abs(d(2))
                    % line slope less than 45 deg
                    dx = sign(d(1));
                    L = bug.Temp_mline;
                    y = -( (robot(1)+dx)*L(1) + L(3) ) / L(2);
                    dy = round(y - robot(2));
                else
                    % line slope greater than 45 deg
                    dy = sign(d(2));
                    L = bug.Temp_mline;
                    x = -( (robot(2)+dy)*L(2) + L(3) ) / L(1);
                    dx = round(x - robot(1));
                end
                % disp(robot + [dx; dy])

                n = robot + [dx; dy];
                % are we on the M-line now ?
                if abs( [robot' 1]*bug.mline') <= 1
                    bug.message('(%d,%d) moving along the M-line', n);
                    % are closer than when we encountered the obstacle?
                    % if colnorm(robot-bug.goal) < colnorm(bug.H(bug.j,:)'-bug.goal)
                        % back to moving along the M-line
                        n = robot;
                        bug.step = 1;
                        bug.j = bug.j + 1;
                        return;
                    % end
                end
            end % step 3

        end % next
        
        function plan(bug)
            error('RTB:Bug2:badcall', 'This class has no plan method');
        end

    end % methods
end % classdef
