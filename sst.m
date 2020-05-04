function [path] = rrt(map, start, goal)
% RRT Find the shortest path from start to goal.
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

    V_active = [];
    V_inactive = [];
    s.witness = start;
    s.rep = start;
    witness = [s];
    
    tStart = tic;
    tEnd = toc(tStart);
    
    while tEnd < 20
        [state, control] = random_sample(map, robot);
        [closest, idx] = nearest_vertex(state, V_active);
        [valid, new, duration] = propogate(closest, control, min_step, max_step, map, robot);
        if valid
            add_to_tree(); 
        end
        tEnd = toc(tStart);
    end

end

function res = add_to_tree(witness, new, duration, delta)

    [witness, new_witness, nearest] = check_for_witness(witness, new, delta);
    if isnan(new_witness.rep) || (new_witness.rep.cost > nearest.cost + duration)
        if isnan(best_goal) || 
            
        end
    end
    
end

function [state, control] = random_sample(map, robot)
    state = [];
    while isRobotCollided(state, map, robot)
        state = [];
        for j = 1:3
            state = [state, robot.lowerLim(j) + (robot.upperLim(j)-robot.lowerLim(j))*rand(1)];
        end
        state = [state,0,0,0];
    end
    control = [0.5 * rand(1), 0.5 * rand(1), 0.5 * rand(1)];
end


function [closest, idx] = nearest_vertex(q, V_active)

    dist = Inf;
    for j = 1:size(V_active, 1)
        node = V_active(j, :);
        temp = norm(q - node);
        if temp < dist
           dist = temp;
           closest = node;
           idx = j;
        end
    end 
end


function [valid, state, duration] = propogate(start, control, min_step, max_step, map, robot)
    
    integration = 0.002;
    steps = randi([min_step, max_step],1);
    valid = true;
    temp = start;
    for i = 1:steps
        temp(1) = temp(1) + integration * control(1);
        temp(2) = temp(2) + integration * control(2);
        temp(3) = temp(3) + integration * control(3);
        temp = bounds(temp, robot);
        valid = valid && isRobotCollided(temp, map, robot);
    end
    state = temp;
    duration = steps * integration;
end

function state = bounds(q, robot)
    
    state = q;
    for i = 1:3
        if q(i) > robot.upperLim(i)
            state(i) = robot.upperLim(i);
            
        elseif q(i) < robot.lowerLim(i)
            state(i) = robot.lowerLim(i);  
        end        
    end

end


function [witness, new_witness, nearest] = check_for_witness(witness, new, delta)
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