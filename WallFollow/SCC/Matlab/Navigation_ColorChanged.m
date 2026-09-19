

classdef Navigation_ColorChanged < handle

    properties
        options
        
        occgrid     % occupancy grid as provided by user
        occgridnav  % inflated occupancy grid
        goal        % goal coordinate
        start       % start coordinate

        verbose     % verbosity
        seed            % current random seed
        spincount

        randstream
        seed0
        
        w2g        % transform from world coordinates to grid coordinates
    end
    
    
    % we make this class abtract
    methods(Abstract)
        plan
        next
    end
    
    methods

        % TODO fix up set methods for goal
        % setup argument callback like features, can we inherit from that.
        % occ grid should be an option

        % constructor

        function nav = Navigation_ColorChanged(varargin)
        

            if nargin >= 1 && ( isnumeric(varargin{1}) || islogical(varargin{1}))
                % first argument is the map
                map = double( varargin{1} );
                varargin = varargin(2:end);
                if isnumeric(map) && ~isscalar(map)
                    nav.occgrid = map;
                    nav.w2g = SE2(0, 0, 0);
                elseif isstruct(map)
                     nav.occgrid = map.map;
                     nav.w2g = nav.T;
                end
            end
            
            % default values of options
            opt.goal = [];
            opt.inflate = 0;
            opt.private = false;
            opt.reset = false;
            opt.seed = [];
            opt.transform = SE2;
            
            [opt,lp.options] = tb_optparse(opt, varargin);

            % optionally inflate the obstacles

            if opt.inflate > 0
                if exist('idilate') == 2
                    % use MVTB
                    nav.occgridnav = idilate(nav.occgrid, kcircle(opt.inflate));
                elseif exist('imdilate') == 2
                    % use IPT
                    nav.occgridnav = imdilate(nav.occgrid, strel('disk',opt.inflate));
                else
                    error('RTB:Navigatio:Navigation', 'Need to have MVTB or IPT installed to perform obstacle inflation');
                end
            else
                nav.occgridnav = nav.occgrid;
            end
            
            % copy other options into the object
            nav.verbose = opt.verbose;
            if ~isempty(opt.goal)
                nav.goal = opt.goal(:)';
            end

            % create a private random number stream if required
            if opt.private
                nav.randstream = RandStream.create('mt19937ar');
            else
                nav.randstream = RandStream.getGlobalStream();
            end

            % reset the random number stream if required
            if opt.reset
                nav.randstream.reset();
            end

            % return the random number stream to known state if required
            if ~isempty(opt.seed)
                set(nav.randstream.set(opt.seed));
            end

            % save the current state in case it later turns out to give interesting results
            nav.seed0 = nav.randstream.State;
            
            nav.w2g = opt.transform;

            nav.spincount = 0;
        end

        function pp = query(nav, start, varargin)
         
            
            opt.animate = false;
            opt = tb_optparse(opt, varargin);
            
            % make sure start and goal are set and valid, optionally prompt
            nav.checkquery(start);
            
            if opt.animate
                nav.plot();
                hold on
            end
            
            % iterate using the next() method until we reach the goal
            robot = nav.start;
            path = nav.start(:);
            while true
                if opt.animate
                    plot(robot(1), robot(2), 'g.', 'MarkerSize', 12);
                    drawnow
                end
                
                % move to next point on path
                robot = nav.next(robot);
                
                % are we there yet?
                if isempty(robot)
                    path = [path nav.goal(:)];
                    % yes, exit the loop
                    break
                else
                    path = [path robot(:)]; % append it to the path
                end
            end
            
            % return the path 
            if nargout > 0
                pp = path';
            end
        end

        function plot(nav, varargin)
      
            nav.plot_bg(varargin{:});
            nav.plot_fg(varargin{:});
        end
        
        function plot_bg(nav, varargin)
       
            
            opt.distance = [];
            opt.colormap = @bone;
            opt.beta = 0.2;
            opt.inflated = false;

            opt = tb_optparse(opt, varargin);
            
            if opt.inflated
                occgrid = nav.occgridnav;
            else
                occgrid = nav.occgrid;
            end
            
            clf
            if isempty(opt.distance) || all(all(~isfinite(opt.distance)))
                % create color map for free space + obstacle:
                %   free space, color index = 1, white, 
                %   obstacle, color index = 2, red
                cmap = [1 1 1; 0.4 0.4 0.4];  % non obstacles are white %%%%%---- The second row determine obstacle`s color
                image(occgrid+1, 'CDataMapping', 'direct', ...
                    'AlphaData', occgrid);
                colormap(cmap)
                
            else
             
                
                % find maximum distance, ignore infinite values in
                % obstacles
                d = opt.distance(isfinite(opt.distance));
                d = d + 2;   % minimum distance is cmap=2 or black
                maxdist = max(d(:));

               
                cmap = [1 0 0; opt.colormap(ceil(maxdist))];
                
                % distance of 0 has display value of 2
                opt.distance = opt.distance + 2;
                
                % invalid distances show as black
                opt.distance(isnan(opt.distance)) = 2;
                
                % ensure obstacles appear as red
                opt.distance(occgrid > 0) = 1;
                
                % display it with colorbar
                image(opt.distance, 'CDataMapping', 'direct');
                set(gcf, 'Renderer', 'Zbuffer')
                colormap(cmap)
                cb = colorbar;
                cb.Label.String = 'Distance to goal (cells)';
                brighten(opt.beta)
            end
            
            % label the grid
            set(gca, 'Ydir', 'normal');
            set(gca,'fontsize',17)
            xlabel('x','FontSize',18);
            ylabel('y','FontSize',18);
            grid on
            hold on
        end
        
        function plot_fg(nav, varargin)
       
            
            opt.pathmarker =  {};
            opt.startmarker = {};
            opt.goalmarker =  {};
            opt.goal = true;
            
            pathmarker =  {'g.', 'MarkerSize', 12};
            startmarker = {'bo','MarkerFaceColor', 'b', 'MarkerEdgeColor', 'w', 'MarkerSize', 12};
            goalmarker =  {'bp', 'MarkerFaceColor', 'b', 'MarkerEdgeColor', 'w', 'MarkerSize', 18};
            
            [opt,args] = tb_optparse(opt, varargin);
                    
                        
            % overlay a path if provided
            if ~isempty(args) && isnumeric(args{1})
                p = args{1};
                if numcols(p) < 2
                    error('expecting Nx2 or Nx3 matrix of points');
                end
                if numcols(p) == 2
                    plot(p(:,1), p(:,2), pathmarker{:}, ...
                        opt.pathmarker{:}, 'Tag', 'path');
                else
                    plot3(p(:,1), p(:,2), p(:,3), pathmarker{:}, ...
                        opt.pathmarker{:}, 'Tag', 'path');
                end
            end
            
            % mark start and goal if requested
            if length(nav.goal) == 2
                if opt.goal && ~isempty(nav.goal)
                    plot(nav.goal(1), nav.goal(2), ...
                        goalmarker{:}, opt.goalmarker{:}, 'Tag', 'goal');
                end
                if opt.goal && ~isempty(nav.start)
                    plot(nav.start(1), nav.start(2), ...
                        startmarker{:}, opt.startmarker{:}, 'Tag', 'start');
                end
            else
                if opt.goal && ~isempty(nav.goal)
                    plot3(nav.goal(1), nav.goal(2), nav.goal(3)+0.1, ...
                        goalmarker{:}, opt.goalmarker{:}, 'Tag', 'goal');
                end
                if opt.goal && ~isempty(nav.start)
                    plot3(nav.start(1), nav.start(2), nav.start(3)+0.1, ...
                        startmarker{:}, opt.startmarker{:}, 'Tag', 'start');
                end
            end

            hold off
        end  
        
        function display(nav)
       
            loose = strcmp( get(0, 'FormatSpacing'), 'loose');
            if loose
                disp(' ');
            end
            disp([inputname(1), ' = '])
            disp( nav.char() );
        end % display()

        function s = char(nav)
      
            s = [class(nav) ' navigation class:'];
            
            s = char(s, sprintf('  occupancy grid: %dx%d', size(nav.occgrid)));
            if ~isempty(nav.goal)
                if length(nav.goal) == 2
                    s = char(s, sprintf('  goal: (%d,%d)', nav.goal) );
                else
                    s = char(s, sprintf('  goal: (%g,%g, %g)', nav.goal) );
                    
                end
            end
        end
        
        
        function setgoal(nav, goal)
            
            if isempty(goal)
                nav.plot();
                disp('select goal location'); beep
                goal = round(ginput(1));
            end
            % make upright
            nav.goal = goal(:);
            
            % check if reachable
            if nav.isoccupied(nav.goal)
                error('Navigation:checkquery:badarg', 'goal location inside obtacle');
            end
        end
        
        function checkquery(nav, start, goal)
            
            % if any of start or goal are [], prompt the user to select
            if isempty(start)
                nav.plot();
                disp('Select start location'); beep
                start = round(ginput(1));
            end
            
            if nargin == 3
                % this planner supports a query with a goal
                if isempty(goal)
                    nav.plot();
                    disp('Select goal location'); beep
                    goal = round(ginput(1));
                end
            end
            
            % make start and goal column vectors
            nav.start = start(:);
            if nargin == 3
                % this planner supports a query with a goal
                nav.goal = goal(:);
            end
            
            % check if reachable
            assert(~nav.isoccupied(nav.start(1:2)), 'Navigation:checkquery:badarg', 'start location inside obstacle');
            
            if nargin == 3
                % make upright
                nav.goal = goal(:);
                
                % check if reachable
                assert(~nav.isoccupied(nav.goal(1:2)), 'Navigation:checkquery:badarg', 'goal location inside obstacle');
            end
        end
        
        
        function occ = isoccupied(nav, x, y)
           
            
            if isempty(nav.occgridnav)
                occ = false;
                return
            end
            
            if nargin == 2
                % isoccupied(p)
                if numel(x) == 2
                    x = x(:);
                end
                assert(size(x,1) == 2, 'RTB:Navigation:isoccupied', 'P must have 2 rows');
                pos = x;
            else
                % isoccupied(x,y)
                assert(numel(x) == numel(y), 'RTB:Navigation:isoccupied', 'X and Y must be same length');
                pos = [x(:)'; y(:)'];
            end
            
            % convert from world coordinates to grid coordinates
            pos = round( nav.w2g * pos );
            
            % find all those that lie in the map
            k = pos(1,:) > 0 & pos(1,:) <= size(nav.occgrid,2) & pos(2,:) > 0 & pos(2,:) <= size(nav.occgrid,1);
            
            % get the indices into the map
            i = sub2ind(size(nav.occgrid), pos(2,k), pos(1,k));
            
            occ = ones(1, size(pos,2), 'logical'); % by default all occupied (true)
            occ(k) = nav.occgridnav(i) > 0;
        end
        
        function goal_change(nav)
            %Navigation.goal_change Notify change of goal
            %
            % Invoked when the goal property of the object is changed.  Typically this
            % is overriden in a subclass to take particular action such as invalidating
            % a costmap.
        end
        
        function navigate_init(nav, start)
         
        end


        function r = rand(nav, varargin)
       
            r = nav.randstream.rand(varargin{:});
        end

        function r = randn(nav, varargin)
       
            r = nav.randstream.randn(varargin{:});
        end

        function r = randi(nav, varargin)
      
            r = nav.randstream.randi(varargin{:});
        end
        
        function verbosity(nav, v)
        %Navigation.verbosity Set verbosity
        %
        % N.verbosity(V) set verbosity to V, where 0 is silent and greater
        % values display more information.
            nav.verbose = v;
        end
       
        
        function message(nav, varargin)
     
            if nav.verbose
                fprintf([class(nav) ' debug:: ' sprintf(varargin{:}) '\n']);
            end
        end
        
                function spinner(nav)
            %Navigation.spinner Update progress spinner
            %
            % N.spinner() displays a simple ASCII progress spinner, a rotating bar.
            spinchars = '-\|/';
            nav.spincount = nav.spincount + 1;
            fprintf('\b%c', spinchars( mod(nav.spincount, length(spinchars))+1 ) );
        end
        
    end
    
    methods (Static)
        
        function show_distance(d)
            d(isinf(d)) = NaN;
            clf
            ax = gca;
            colormap(gray(256));

            ax.CLimMode = 'Manual';
            ax.CLim = [0 max(d(:))];
            image(d, 'CDataMapping', 'scaled');
            ax.YDir = 'normal';
            grid on; xlabel('X'); ylabel('Y');
            drawnow
        end

        function h = progress_init(title)
            h = waitbar(0, title, ...
                'CreateCancelBtn', 'setappdata(gcbf, ''canceling'', 1)');
        end
        
        function progress(h, x)
            waitbar(x, h);
        end
        
        function progress_delete(h)
            delete(h);
        end

    end % method

end % classde