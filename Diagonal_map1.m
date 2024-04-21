function A_star_pathfinding
    tic
    % Environment size
    MAX_ROWS = 40;
    MAX_COLS = 40;
    TOTAL_NODES = MAX_ROWS * MAX_COLS;
    LARGE_VALUE = 9999;

    % Start and goal positions
    startX = 1; startY = 1;
    goalX = 40; goalY = 40;

    % Obstacles [x1, y1, width, height]
    obstaclesrec = {[5,5,6,2],[12,15,3,10],[16,8,1,12],[30,20,4,15],[17,1,6,19],[7,30,10,2],[23,27,2,13]};

    % Define object size
    objectSize = 4; % Considering an object of 4*4

    % Perform A* search and receive parents array for path plotting
    [parents, searchArea] = aStarSearch(startX, startY, goalX, goalY, MAX_COLS, MAX_ROWS, TOTAL_NODES, LARGE_VALUE, obstaclesrec);
    toc

    % Plot the environment, obstacles, and the path
    plotEnvironmentAndPath(parents, searchArea, startX, startY, goalX, goalY, MAX_COLS, obstaclesrec, objectSize);
end

function idx = xyToIndex(x, y, MAX_COLS)
    idx = (y - 1) * MAX_COLS + (x - 1) + 1;
end

function [x, y] = indexToXY(idx, MAX_COLS)
    x = mod(idx - 1, MAX_COLS) + 1;
    y = floor((idx - 1) / MAX_COLS) + 1;
end

function distance = calculateDistance(x1, y1, x2, y2)
    dx = abs(x1 - x2);
    dy = abs(y1 - y2);
    D = 1; % Cost of moving horizontally or vertically
    D2 = sqrt(2); % Cost of moving diagonally (can be the same as D for grids where diagonal moves are allowed)
    distance = D * (dx + dy) + (D2 - 2 * D) * min(dx, dy);
end

% When not considering obstacle expansion
% function isValid = isValidPosition(x, y, MAX_COLS, MAX_ROWS, obstaclesrec)
%     isValid = x >= 1 && x <= MAX_COLS && y >= 1 && y <= MAX_ROWS;
%     for i = 1:length(obstaclesrec)
%         obstacle = obstaclesrec{i};
%         if x >= obstacle(1) && x <= obstacle(1) + obstacle(3) && ...
%            y >= obstacle(2) && y <= obstacle(2) + obstacle(4)
%             isValid = false;
%             break;
%         end
%     end
% end

function isValid = isValidPosition(x, y, MAX_COLS, MAX_ROWS, obstaclesrec)
    isValid = x >= 1 && x <= MAX_COLS && y >= 1 && y <= MAX_ROWS;
    expansion = 2; % Expand obstacle area by 2 nodes on each side

    for i = 1:length(obstaclesrec)
        obstacle = obstaclesrec{i};
        % Adjust obstacle coordinates to include the expansion
        obstacleExpanded = [obstacle(1) - expansion, obstacle(2) - expansion, ...
                            obstacle(3) + 2*expansion, obstacle(4) + 2*expansion];

        if x >= obstacleExpanded(1) && x <= obstacleExpanded(1) + obstacleExpanded(3) && ...
           y >= obstacleExpanded(2) && y <= obstacleExpanded(2) + obstacleExpanded(4)
            isValid = false;
            break;
        end
    end
end

function [openList, closedList, g, h, f, parents] = initializeGrid(TOTAL_NODES, LARGE_VALUE)
    openList = false(1, TOTAL_NODES);
    closedList = false(1, TOTAL_NODES);
    g = ones(1, TOTAL_NODES) * LARGE_VALUE;
    h = ones(1, TOTAL_NODES) * LARGE_VALUE;
    f = ones(1, TOTAL_NODES) * LARGE_VALUE;
    parents = -ones(1, TOTAL_NODES);
end

function [parents, searchArea] = aStarSearch(startX, startY, goalX, goalY, MAX_COLS, MAX_ROWS, TOTAL_NODES, LARGE_VALUE, obstaclesrec)
    [openList, closedList, g, h, f, parents] = initializeGrid(TOTAL_NODES, LARGE_VALUE);

    searchArea = false(1, TOTAL_NODES); % Initialize search area tracking
    
    startIdx = xyToIndex(startX, startY, MAX_COLS);
    goalIdx = xyToIndex(goalX, goalY, MAX_COLS);
    g(startIdx) = 0;
    h(startIdx) = calculateDistance(startX, startY, goalX, goalY);
    f(startIdx) = h(startIdx);
    openList(startIdx) = true;

    while true
        currentIdx = -1;
        minF = LARGE_VALUE;
        for i = 1:TOTAL_NODES
            if openList(i) && f(i) < minF
                minF = f(i);
                currentIdx = i;
            end
        end

        if currentIdx == -1
            disp('Path not found.');
            return;
        end

        if currentIdx == goalIdx
            return; % Parents array will be used for path reconstruction
        end

        openList(currentIdx) = false;
        closedList(currentIdx) = true;

        [currentX, currentY] = indexToXY(currentIdx, MAX_COLS);

        % Consider all eight directions
        for dx = -1:1
            for dy = -1:1
                if dx == 0 && dy == 0
                    continue;  % Skip the current node itself
                end
                
                newX = currentX + dx;
                newY = currentY + dy;
        
                % Boundary and obstacle check
                if newX < 1 || newX > MAX_COLS || newY < 1 || newY > MAX_ROWS || ~isValidPosition(newX, newY, MAX_COLS, MAX_ROWS, obstaclesrec)
                    continue;
                end
                
                neighborIdx = xyToIndex(newX, newY, MAX_COLS);
                if closedList(neighborIdx)
                    continue;  % Skip if neighbor is in the closed list
                end
        
                % Calculate g score for the neighbor
                if dx ~= 0 && dy ~= 0
                    % Diagonal movement
                    tentativeG = g(currentIdx) + sqrt(2);
                else
                    % Orthogonal movement
                    tentativeG = g(currentIdx) + 1;
                end
        
                % Continue with the rest of your A* logic...
                if ~openList(neighborIdx) || tentativeG < g(neighborIdx)
                    parents(neighborIdx) = currentIdx;
                    g(neighborIdx) = tentativeG;
                    h(neighborIdx) = max(abs(newX - goalX), abs(newY - goalY));  % Chebyshev distance for heuristic
                    f(neighborIdx) = g(neighborIdx) + h(neighborIdx);
                    openList(neighborIdx) = true;
                    searchArea(neighborIdx) = true;
                end
            end
        end
    end
    return; % Return both parents and searchArea for plotting
end

function plotEnvironmentAndPath(parents, searchArea, startX, startY, goalX, goalY, MAX_COLS, obstaclesrec, objectSize)
    [pathX, pathY, totalPathDistance] = reconstructPath(parents, goalX, goalY, startX, startY, MAX_COLS);

    figure; hold on; axis equal; grid on; axis([1 40 1 40]);
    title('A* Pathfinding - Diagonal Distance'); xlabel('X'); ylabel('Y');

    % Convert searchArea into (x, y) coordinates for plotting
    [searchY, searchX] = find(searchArea);
    for i = 1:length(searchX)
        [x, y] = indexToXY(searchX(i), MAX_COLS);
        plot(x, y, '.', 'MarkerEdgeColor', [0.5 0.5 0.5], 'MarkerSize', 10);
    end

    % Plot expanded obstacles
    for i = 1:length(obstaclesrec)
            rect = obstaclesrec{i};
            % Calculate the center of the rectangle for plotting
            centerX = rect(1) + rect(3)/2;
            centerY = rect(2) + rect(4)/2;
            expansion = floor(objectSize/2);
            % Calculate expanded rectangle parameters
            expandedX = centerX - (rect(3)/2 + expansion);
            expandedY = centerY - (rect(4)/2 + expansion);
            expandedWidth = rect(3) + 2*expansion;
            expandedHeight = rect(4) + 2*expansion;
            % Plot a rectangle or use a scatter plot if you want to match the searched area style more closely
            % Here we use scatter for individual points to closely mimic the 'yo' marker style but in light blue
            [X, Y] = meshgrid(expandedX:expandedX+expandedWidth, expandedY:expandedY+expandedHeight);
            scatter(X(:), Y(:), 'MarkerEdgeColor', [1, 0, 0], 'Marker', '.', 'SizeData', 100); % Light blue color
            hold on;
    end
    
    % Plot obstacles
    for i = 1:length(obstaclesrec)
        obstacle = obstaclesrec{i};
        rectangle('Position', [obstacle(1), obstacle(2), obstacle(3), obstacle(4)], 'FaceColor', [0 0 0]); % black color
    end
    
    % Plot path
    plot(pathX, pathY, 'b-o', 'LineWidth', 2);
    
    % Plot start and goal points
    plot(startX, startY, 'go', 'MarkerSize', 10, 'MarkerFaceColor', [0 1 0]); % green color
    plot(goalX, goalY, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', [1 0 0]); % red color

    fprintf('Total path distance: %.2f units\n', totalPathDistance);
    
    hold off;
end

function [pathX, pathY, totalPathDistance] = reconstructPath(parents, goalX, goalY, startX, startY, MAX_COLS)
    idx = xyToIndex(goalX, goalY, MAX_COLS);
    pathX = goalX; % Initialize with the goal position
    pathY = goalY; % Initialize with the goal position
    totalPathDistance = 0; % Initialize total path distance
    
    while parents(idx) ~= -1
        nextIdx = parents(idx);
        [nextX, nextY] = indexToXY(nextIdx, MAX_COLS);
        
        % Determine if the movement is diagonal or orthogonal by comparing X and Y differences
        if abs(nextX - pathX(1)) == 1 && abs(nextY - pathY(1)) == 1
            totalPathDistance = totalPathDistance + sqrt(2); % Diagonal movement
        else
            totalPathDistance = totalPathDistance + 1; % Orthogonal movement
        end
        
        % Prepend the new position to the path
        pathX = [nextX, pathX];
        pathY = [nextY, pathY];
        
        idx = nextIdx; % Move to the next node in the path
    end
    
    % No need to add the start position to the beginning
    % because we start from the goal
end