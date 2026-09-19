    %BUG1 Bug navigation class
    
    classdef Bug1 < Navigation_ColorChanged
    
        properties(Access=protected)
            H       % hit points
            j       % number of hit points
            mline   % line from starting position to goal
            step    % state, in step 1 or step 2 of algorithm
            edge    % edge list
            k       % edge index
            C       % closest point
            closestPoint_k  % closest point index (order number in edge list)
        end
    
        methods
    
            function bug = Bug1(varargin)
               
                % invoke the superclass constructor
                bug = bug@Navigation_ColorChanged(varargin{:});
    
                bug.H = [];
                bug.j = 1;
                bug.step = 1;
            end
    
            function pp = query(bug, start, goal, varargin)
               
                disp('Bug1 is running ... ')
             
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
                        if bug.step == 3
                            plot(robot(1), robot(2), 'r.', 'MarkerSize', 12);
                        else
                            plot(robot(1), robot(2), 'b.', 'MarkerSize', 12);
                        end
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
                        bug.C = robot; % current closest point to the goal
                        bug.step = 2;
                        % get a list of all the points around the obstacle
                        bug.edge = edgelist(bug.occgridnav == 0, robot);
                        bug.k = 2;  % skip the first edge point, we are already there
                    else
                        n = robot + [dx; dy];
                    end
                end % step 1


                if bug.step == 2
                    % Step 2.  Move around the obstacle until we reach a point we started.
                    if colnorm(bug.goal-robot) == 0 % are we there yet?
                        return
                    end
    
                    % are closer than when we encountered the obstacle?
                    if colnorm(robot-bug.goal) < colnorm(bug.H(bug.j,:)'-bug.goal)
                        % are closer than when we encountered the obstacle?
                        if colnorm(robot-bug.goal) < colnorm(bug.C-bug.goal)
                            % back to moving along the M-line
                            bug.C = robot;
                            bug.closestPoint_k = bug.k;
                        end
                    end

                    if bug.k <= numcols(bug.edge)
                        n = bug.edge(:,bug.k);  % next edge point
                    elseif isequal(robot', bug.H(bug.j,:)) || (colnorm(robot-bug.H(bug.j,:)') < 2)
                        % Determine the direction (shorter path) to return to the closest point to the goal 
                        if bug.closestPoint_k > size(bug.edge,2)/2
                            bug.edge = fliplr(bug.edge);
                        end
                        bug.k = 0;
                        bug.step = 3;
                        bug.mline = homline(bug.C(1), bug.C(2), ...
                            bug.goal(1), bug.goal(2));
                        bug.mline = bug.mline / norm(bug.mline(1:2));
                    else
                        % we are at the end of the list of edge points, we
                        % are back where we started.  Step 2.c test.
                        error('RTB:bug2:noplan', 'robot is trapped (step 2)')
                        return;
                    end

                    % no, keep going around
                    bug.message('(%d,%d) keep moving around obstacle', n)
                    bug.k = bug.k+1;
                end % step 2
    

                if bug.step == 3
                    % Step 3.  return to the closest point to th goal
                    if colnorm(bug.goal-robot) == 0 % are we there yet?
                        return
                    end
    
                    if bug.k <= numcols(bug.edge)
                        n = bug.edge(:,bug.k);  % next edge point
                    else
                        % we are at the end of the list of edge points, we
                        % are back where we started.  Step 2.c test.
                        error('RTB:bug2:noplan', 'robot is trapped (step 3)')
                        return;
                    end
    
                    % are we on the closest point now ?
                    if isequal(robot, bug.C)
                        bug.message('(%d,%d) Return to the closest point', n);
                        % back to moving along the M-line
                        bug.j = bug.j + 1;
                        bug.step = 1;
                        % bug.plot_mline();
                        return;
                    end

                    % no, keep going around
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
