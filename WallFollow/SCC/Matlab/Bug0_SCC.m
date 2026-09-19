    %BUG0 Bug navigation class 
    
    
    classdef Bug0_SCC < Navigation_ColorChanged
    
        properties(Access=protected)
            H       % hit points
            j       % number of hit points
            mline   % line from starting position to goal
            step    % state, in step 1 or step 2 of algorithm
            edge    % edge list
            k       % edge index
            straight_line   % to move staright
            temp_goal   % point to reach which is "len" far from current position
            len  % max length to go forward on temp-mline
            prev_robot  % to store the previous coordinates
            flag    % faqat ona goraki step 3 dan step 1 a gechanda, gechmamish o axir nuxtani print eliya
        end
    
        methods
    
            function bug = Bug0_SCC(varargin)
                %Bug2.Bug2 Construct a Bug2 navigation object 
                %
                % B = Bug2(MAP, OPTIONS) is a bug2 navigation object, and MAP is an occupancy grid,
                % a representation of a planar world as a matrix whose elements are 0 (free
                % space) or 1 (occupied).
                %
                % Options::
                % 'goal',G      Specify the goal point (1x2)
                % 'inflate',K   Inflate all obstacles by K cells.
                %
                % See also Navigation.Navigation.
    
                % invoke the superclass constructor
                bug = bug@Navigation_ColorChanged(varargin{:});
    
                bug.H = [];
                bug.j = 1;
                bug.step = 1;
                bug.len = 5;
                bug.flag = 1; 
            end
    
            function pp = query(bug, start, goal, radius, varargin)
               

                disp('Bug0_NBHCM is running ... ')
             
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
                        plot(robot(1), robot(2), 'b.', 'MarkerSize', 12);
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
                    robot = bug.next(robot, radius);
    
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
            
            function n = next(bug, robot, radius)
                
                % implement the main state machine for bug2
                n = [];
                robot = robot(:); % these are coordinates (x,y)            
                
                if bug.step == 1
                    % disp(bug.step)
                    % Step 1.  Move along the M-line toward the goal

                    if colnorm(bug.goal - robot) == 0 % are we there yet?
                        return
                    end

                    if bug.flag == 1
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
                            bug.flag = 0;
                            % get a list of all the points around the obstacle
                            bug.edge = edgelist(bug.occgridnav == 0, robot);
                            % 1: construct the mline parallel to the obstacle wall (using the second and third points on the edge)
                            % check if two first point of the edge list are on a vertical or horizontal line, if not take the
                            % first and last pint of the edge list for constructing mline parallel to the obstacle wall
                            if (bug.edge(1,1)-bug.edge(1,2)==0) || (bug.edge(2,1)-bug.edge(2,2)==0)
                                point1 = bug.edge(:,1);
                                point2 = bug.edge(:,2);
                            else
                                point1 = bug.edge(:,end);
                                point2 = bug.edge(:,1);
                            end
                            bug.straight_line = homline(point1(1), point1(2), ...
                                point2(1), point2(2));
                            bug.straight_line = bug.straight_line / norm(bug.straight_line(1:2));
                            % bug.plot_Temp_mline();

                            % 2: construct new temp mline to walk for the predetermined length (len)
                            % Calculate the direction vector from the starting point to the first point in A
                            dx = bug.edge(1,2) - bug.edge(1,1);
                            dy = bug.edge(2,2) - bug.edge(2,1);
                            % Calculate the distance from the starting point to the first point in A
                            d = sqrt(dx^2 + dy^2);
                            % Calculate the coordinates of the m-th point on the line
                            bug.temp_goal(1) = bug.edge(1,1) + bug.len * (dx / d);
                            bug.temp_goal(2) = bug.edge(2,1) + bug.len * (dy / d);
                            bug.temp_goal = bug.temp_goal';
                            % Display the coordinates of the desired point
                            % fprintf('Coordinates of the %d -th point on the line: %.2f, %.2f\n', bug.len, new_x, new_y);
                            n=robot + [dx; dy];
                            bug.prev_robot = robot;
                            bug.k = 2;  % skip the first edge point, we are already there
                        else
                            n = robot + [dx; dy];
                        end

                    else
                        bug.flag = 1;
                        n = bug.prev_robot;
                    end

                end % step 1

                if bug.step == 2 
                    % Step 2.  Move around the obstacle until we reach a point
                    % on the M-line closer than when we started.
     % disp(bug.step)

                    if colnorm(bug.goal-robot) == 0 % are we there yet?
                        return
                    end

                    if ~iscolumn(bug.temp_goal)
                        bug.temp_goal = bug.temp_goal';
                    end

                    d = bug.temp_goal-robot;
                    if abs(d(1)) > abs(d(2))
                        % line slope less than 45 deg
                        dx = sign(d(1));
                        L = bug.straight_line;
                        y = -( (robot(1)+dx)*L(1) + L(3) ) / L(2);
                        dy = round(y - robot(2));
                    else
                        % line slope greater than 45 deg
                        dy = sign(d(2));
                        L = bug.straight_line;
                        x = -( (robot(2)+dy)*L(2) + L(3) ) / L(1);
                        dx = round(x - robot(1));
                    end
                    n = robot + [dx; dy];

                    if colnorm(bug.temp_goal-robot) == 0 
                        bug.edge = circleedgelist(robot', bug.prev_robot', radius);
                        bug.k = 1;  % skip the first edge point, we are already there
                        bug.step = 3;
                    else
                        bug.prev_robot = robot;
                    end

                    bug.message('(%d,%d) keep moving around obstacle', n)
                    bug.k = bug.k+1;
                end % step 2

                
                if bug.step == 3
       % disp(bug.step)
                     % on the M-line closer than when we started.
                    if colnorm(bug.goal-robot) == 0 % are we there yet?
                        return
                    end
    
                    if bug.k <= numcols(bug.edge)
                        n = bug.edge(:,bug.k);  % next edge point
                        if bug.isoccupied(n)
                            % disp('here')
                            bug.mline = homline(n(1), n(2), ...
                                bug.goal(1), bug.goal(2));
                            bug.mline = bug.mline / norm(bug.mline(1:2));
                            bug.step = 1;
                            n = bug.edge(:,bug.k); % bu va bunnan soraki satr + 290 va 291 minci satrlar faqat ona goradilar ki step3 un manea daymax nuxtasi print olsun sora getsin step1 a
                            bug.prev_robot = bug.edge(:,bug.k-1);
                            bug.k = 2;
                        end
                    else
                        % we are at the end of the list of edge points, we
                        % are back where we started.  Step 2.c test.
                        error('RTB:bug2:noplan', 'robot is trapped')
                        return;
                    end
                    bug.message('(%d,%d) keep moving around obstacle', n)
                    bug.k = bug.k+1;
                end % step 3

                

                % grid off
            end % next
            
            function plan(bug)
                error('RTB:Bug2:badcall', 'This class has no plan method');
            end
    
        end % methods
    end % classdef
