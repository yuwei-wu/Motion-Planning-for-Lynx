function [path] = sst(map, start, goal)
% SST Find the shortest path from start to goal.
%   PATH = rrt(map, start, goal) returns an mx6 matrix, where each row
%   consists of the configuration of the Lynx at a point on the path. The
%   first row is start and the last row is goal. If no path is found, PATH
%   is a 0x6 matrix.
%
% INPUTS:
%   map     - the map object to plan in
%   start   - 1x6 vector of the starting configuration
%   goal:   - 1x6 vector of the goal configuration


%% Prep Code

    path = [];

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%                  Algortihm Starts Here             %%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    load 'robot.mat' robot  %load robot infos

    % step one: check whether the start and goal position are valid
    if isRobotCollided(start, map, robot) || isRobotCollided(goal, map, robot)
        disp("Your start/goal positions are invalid. Try again.")
        return
    end
    tol = 0.4; % tolerance  of distance to goal 
    best_goal = nan; % closest node to goal
    
    % initialize a tree node struct, with start as the root bode
    count = 1;
    root.point = start;
    root.inactive = false;
    root.parent = nan;
    root.cost = 0;
    root.children = [];
    root.id = count;
    V_active = containers.Map('KeyType','double','ValueType','any');
    V_active(count) = root;
    
    % initialize witness set, used for pruning later
    s.point = root.point;
    s.rep = root;
    witness = [s];
    
    % set timer and stop the algorithm after certain time
    tStart = tic;
    tEnd = toc(tStart);
    
    while tEnd < 30
        [state, control] = random_sample(map, robot);
        nearest = nearest_vertex(state, V_active, 0.3);
        
        % find node with lowest path cost
        cost = Inf;
        for i = 1:length(nearest)
            if nearest(i).cost < cost
                cost = nearest(i).cost;
                closest = nearest(i); 
            end
        end
        
        [valid, new, duration] = propogate(closest.point, control, 200, 500, map, robot);
        if valid
            [witness, new_witness] = check_for_witness(witness, new, 0.2);
            
            % decide if new node should be added
            if ~isstruct(new_witness.rep) || (new_witness.rep.cost > closest.cost + duration)
                if ~isstruct(best_goal) || closest.cost+duration <= best_goal.cost
                    new_node.point = new;
                    new_node.parent = closest;
                    new_node.cost = closest.cost + duration;
                    new_node.inactive = false;
                    new_node.children = [];
                    count = count + 1;
                    new_node.id = count;
                    closest.children = [closest.children new_node];
                    
                    % prune tree for sparsity
                    if ~isstruct(best_goal) && norm(new-goal) < tol
                        best_goal = new_node;
                        V_active = branch_and_bound(root, best_goal, V_active);
                        
                    elseif isstruct(best_goal) && best_goal.cost > new_node.cost && norm(new-goal) < tol
                        best_goal = new_node;
                        V_active = branch_and_bound(root, best_goal, V_active);
                    end
                    
                    % prune nodes dominated by others
                    if isstruct(new_witness.rep)
                        if ~new_witness.rep.inactive
                            new_witness.rep.inactive = true;
                            remove(V_active, new_witness.rep.id);   
                        end
                        
     
                    end
        
                    V_active(count) = new_node;
                                 
                end
            end
        end
        tEnd = toc(tStart);
    end
    
    % construct path backwards from the node closest to goal
    nearest = best_goal;
    while isstruct(nearest.parent)
        path = [path;nearest.point];
        nearest = nearest.parent;
    end
    path = flipud(path);
    path = [root.point;path;goal];
end


function [state, control] = random_sample(map, robot)
    
    % helper function to randomly sample a state in configuration space and
    % a set of control inputs for propagation
    
    % map     - the map object to plan in
    % robot   - robot specification
    
    state = [];
    first = true;
    while first || isRobotCollided(state, map, robot)
        first = false;
        state = [];
        for j = 1:3
            state = [state, robot.lowerLim(j) + (robot.upperLim(j)-robot.lowerLim(j))*rand(1)];
        end
        state = [state,0,0,0];
    end
    control = [0.5 * (rand(1)*2-1), 0.5 * (rand(1)*2-1), 0.5 * (rand(1)*2-1)];
end


function closest = nearest_vertex(q, V_active, delta)

    % find all nodes within a certain distance to a speficied node from
    % active set
    
    % q - query node
    % V_active - set of active nodes
    % delta - distance threshold
    
    closest = [];
    dist = Inf;
    for j = 1:length(V_active)
        node = V_active(j).point;
        temp = norm(q - node);
        if temp < dist
           dist = temp;
           close = V_active(j);
        end
        if temp < delta
           closest = [closest V_active(j)];
        end
    end
    if isempty(closest)
       closest = [close]; 
    end
end


function [valid, state, duration] = propogate(start, control, min_step, max_step, map, robot)
    
    % monte carlo propagate from a start point with control inputs and
    % integration time
    
    % start - start position
    % control - control inputs
    % min_step - minimum step to perform integration
    % max_step - maximum step to perform integration
    % map     - the map object to plan in
    % robot   - robot specification
    
    integration = 0.01;
    steps = randi([min_step, max_step],1);
    valid = true;
    temp = start;
    for i = 1:steps
        temp(1) = temp(1) + integration * control(1);
        temp(2) = temp(2) + integration * control(2);
        temp(3) = temp(3) + integration * control(3);
        temp = bounds(temp, robot);
        valid = valid && ~isRobotCollided(temp, map, robot);
    end
    state = temp;
    duration = steps * integration;
end

function state = bounds(q, robot)

    % enforce physical bounds on each joint of the robot
    
    % q - current configuration
    % robot - robot specification
    
    state = q;
    for i = 1:3
        if q(i) > robot.upperLim(i)
            state(i) = robot.upperLim(i);
            
        elseif q(i) < robot.lowerLim(i)
            state(i) = robot.lowerLim(i);  
        end        
    end

end


function [witness, new_witness] = check_for_witness(witness, new, delta)

    % check if a witness exists within some distance of a newly propagated
    % node
    
    % witness - set of all witnesses
    % new - new node
    % delta - distance threshold
    min_dist = Inf;
    for i = 1:length(witness)
        temp = norm(new - witness(i).point);
        if temp < min_dist
            min_dist = temp;
            nearest = witness(i);
        end
    end
    
    if min_dist > delta
        new_witness.point = new;
        new_witness.rep = nan;
        witness = [witness new_witness];
    else
        new_witness = nearest;
    end
end


function flag = along_best(best_goal, v)
    
    % check if a node is along the path to goal
    
    % best_goal - current best node
    % v - query node
    if ~isstruct(best_goal)
       flag = false; 
    end
    
    while isstruct(best_goal.parent)
        if best_goal.id == v.id
           flag = true;
           return
        end
        best_goal = best_goal.parent;
    end

end

function V_active = branch_and_bound(node, best_goal, V_active)

    % prune leaf nodes with higher path cost than current best node from
    % root
    
    % node - root node to start pruning from
    % best_goal - current best node
    % V_active - set of active nodes
    children = node.children;
    for i = 1:length(children)
       V_active = branch_and_bound(children(i),best_goal, V_active);
    end
    
    if isempty(node.children) && node.cost > best_goal.cost
        
        node.parent.children(node.parent.children.id == node.id) = [];
        if ~node.inactive
            remove(V_active, node.id)
        end
        
    end
end