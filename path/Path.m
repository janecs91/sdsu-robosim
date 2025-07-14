classdef Path
    properties(Constant)
        verbose = false;
    end
    properties
        fileAddress;
        pathName;
        pathPoints;
    end
    methods
        function obj = Path(fileAddress, directory, pathArgs)
            if nargin > 0
                pathAddress = fileAddress;
            else
                pathAddress = 'path_straight.mat';
            end
            pathAddress = sprintf('%s/%s.mat\n', directory, fileAddress);
            if nargin < 2
                pathArgs = {};
            end
            % load terrain
            pathGenerator = PathGenerator(directory);
            try
                pathData = load(pathAddress);
                fprintf("Successfully loaded path: %s\n", pathAddress)
            catch
                fprintf('error: could not find path %s\n', pathAddress)
                disp('generating...');
                disp(pathArgs)
                % generate if doesn't exist
                if size(pathArgs,2) > 0
                    pathAddress = pathGenerator.gen_from_options(pathArgs{:});
                else
                    pathAddress = pathGenerator.gen_from_string(pathAddress);
                end
                pathData = load(pathAddress);
            end
            obj.fileAddress = pathAddress;
            obj.pathName = pathGenerator.get_name(pathArgs{:});
            obj.pathPoints = pathData.pathPoints;
        end
        function obj = add_points(obj, points)
            if size(points, 2) == 2
                obj.pathPoints = [obj.pathPonts; points];
            else
                disp("Need 2 columns (x,y)");
            end
        end
        function point = get_first_point(obj)
            point = obj.pathPoints(1,:);
        end
        function point = get_last_point(obj)
            point = obj.pathPoints(end,:);
        end
        function point = get_point(obj, index)
            point = obj.pathPoints(index,:);
        end
        function count = get_path_points_count(obj)
            count = size(obj.pathPoints,1);
        end
        function name = get_name(obj)
            name = obj.pathName;
        end
        function [width, height, maxY, minY] = get_size(obj)
            width = obj.pathPoints(end,1)-obj.pathPoints(1,1);
            maxY = max(obj.pathPoints(:,2));
            minY = min(obj.pathPoints(:,2));
            height = maxY - minY;
        end
        
        %% points
        function [angle, slope] = get_gamma_at_index(obj, pathPointIndex)
            slope = 0;
            currentPoint = obj.pathPoints(pathPointIndex,:);
            if pathPointIndex < size(obj.pathPoints, 1)
                futurePoint = obj.pathPoints(pathPointIndex+1,:);
                slope = (futurePoint(2)-currentPoint(2))/(futurePoint(1)-currentPoint(1));
            end
            angle = atan(slope);
        end
        function [angle, slope] = get_gamma_at_x(obj, x, minIndex)
            % not done
            y = obj.get_y_at_x(x, minIndex);
            slope = 0;
            currentPoint = obj.pathPoints(pathPointIndex,:);
            if pathPointIndex < size(obj.pathPoints, 1)
                futurePoint = obj.pathPoints(pathPointIndex+1,:);
                slope = (futurePoint(2)-currentPoint(2))/(futurePoint(1)-currentPoint(1));
            end
            slope = (futurePoint(2)-currentPoint(2))/(futurePoint(1)-currentPoint(1));
            angle = atan(slope);
        end
        function distance = compare_distance_to_point_index(obj, index, x1, y1)
            pathPoint = obj.pathPoints(index,:);
            x2 = pathPoint(1);
            y2 = pathPoint(2);
            distance = sqrt((x1-x2)^2+(y1-y2)^2);
        end
        function distance = compare_x_distance_to_point_index(obj, index, x1)
            x2 = obj.pathPoints(index,1);
            distance = abs(x1-x2);
        end
        function [nearestPathPointIndex, nearestPoint] = get_nearest_point_index(obj, x1, y1)
            if nargin < 3
                y1 = 0;
            end
            nearestPathPointIndex = 1;
            smallestDistance = obj.compare_distance_to_point_index(1, x1, y1);
            for i=1:size(obj.pathPoints,1)
                pathPointIndex = i;
                distance = obj.compare_distance_to_point_index(i, x1, y1);
                if distance < smallestDistance
                    nearestPathPointIndex = pathPointIndex;
                    smallestDistance = distance;
                end
                if distance > smallestDistance
                    break
                end
            end
            nearestPoint = obj.pathPoints(nearestPathPointIndex,:);
        end  
        function [futurePoint, futurePathIndex] = get_next_nearest_point(obj, x1, y1, lastIndexNotInUse, accuracyRadius)
            if Path.verbose == true
                disp("GNNP====")
                fprintf("input - x1: %.2f, y1: %.2f, lastind: %d, rad: %.2f\n", x1, y1, lastIndexNotInUse, accuracyRadius);
            end
            % Get last Index (maybe last index-1 if nearest pt is not exact?)
            [lastIndex, nearestPoint] = get_nearest_point_index(obj, x1, y1);
            % Find next path point that is radius distance away
            futurePoints = obj.pathPoints(lastIndex:end,:);
            for i=1:size(futurePoints,1)
                futurePathIndex = lastIndex+i-1;
                futurePoint = futurePoints(i,:);
                x2 = futurePoint(1);
                y2 = futurePoint(2);
                distance = sqrt((x1-x2)^2+(y1-y2)^2);
                if Path.verbose == true
                    fprintf("FPI: %d, FP: %0.2f %0.2f, dist: %.2f\n", futurePathIndex, futurePoint, distance);
                end
                if distance > accuracyRadius
                    break
                end
            end
            overshot = distance - accuracyRadius;
            prevIdx = max(1, futurePathIndex-1);
            %vector = [x2-x1 y2-y1];
            %magnitude = sqrt(vector(1)^2+vector(2)^2);
            %unitVector = vector/magnitude;
            unitVector = Line.get_unit_vector(x1, y1, x2, y2);
            %futurePt = obj.pathPoints(prevIdx,1:2)+overshot*unitVector;
            futurePt = obj.pathPoints(futurePathIndex,1:2)-overshot*unitVector;
            futurePoint = futurePt;
            futurePathIndex = prevIdx;
            if Path.verbose == true
                fprintf("(next nearest pt) path pt: %.2f %.2f, pathidx: %d, x1 y1: %.2f %.2f, x2 y2: %.2f %.2f\n", futurePoint, futurePathIndex, x1, y1, x2, y2);
                fprintf("overshot: %.2f, distance: %.2f, accRad: %.2f\n", overshot, distance, accuracyRadius);
            end
        end
        function [pathPoint, pathPointAngle, pathPointSlope] = get_path_point_at_x(obj, x2, y2)
            [nearestPathPointIndex, nearestPoint] = obj.get_nearest_point_index(x2, y2);
            pathPointIndex = nearestPathPointIndex;
            if x1 > 1 && x1 < nearestPoint(1)
                pathPointIndex = nearestPathPointIndex - 1;
            end
            [slope, angle] = obj.get_gamma_at_index(pathPointIndex);
            x1 = nearestPoint(1);
            y1 = nearestPoint(2);
            pathY = slope(x2-x1)+y1;
            pathPoint = [x1 pathY];
        end
        
        %% nearest point calculations
        function [nearestIndex, smallestDistance] = get_nearest_index_to_x(obj, x, minIndex)
            nearestIndex = minIndex;
            smallestDistance = obj.compare_x_distance_to_point_index(minIndex, x);
            for i=minIndex:size(obj.pathPoints,1)
                distance = obj.compare_x_distance_to_point_index(i, x);
                if distance < smallestDistance
                    nearestIndex = i;
                    smallestDistance = distance;
                end
                if distance > smallestDistance
                    break
                end
            end
        end
        function [y, gamma] = get_y_and_gamma_at_x(obj, x, minIndex)
            if nargin < 3
                minIndex = 1;
            end
            [nearestIndex, smallestDistance] = obj.get_nearest_index_to_x(x, minIndex);
            point1 = obj.pathPoints(nearestIndex,:);
            overshot = x-point1(1);
            if overshot ~= 0
                lineDirection = sign(overshot);
                point2 = obj.pathPoints(nearestIndex+lineDirection,:);
                unitVector = Line.get_unit_vector(point1(1), point1(2), point2(1), point2(2));
                pathPointAtX = point1+overshot*unitVector;
                y = pathPointAtX(2);
                % point(1) should be == x: check this
            else
                lineDirection = 1;
                point2 = obj.pathPoints(nearestIndex+1,:);
                y = point1(2);
            end
            slope = lineDirection*(point2(2)-point1(2))/(point2(1)-point1(1));
            gamma = atan(slope);
        end
        
        
        
        
        function [plusPoint, minusPoint] = get_parallel_point_at_x(obj, x1, y1, distance, index)
            [angle, slope] = obj.get_gamma_at_index(index);
            [plusPoint, minusPoint] = Line.get_parallel_point_from_distance(x1, y1, angle, distance);
        end
        function [plusPoint, minusPoint] = get_parallel_point_at_index(obj, index, distance)
            currentPoint = obj.pathPoints(index,:);
            [angle, slope] = obj.get_gamma_at_index(index);
            x = currentPoint(1);
            y = currentPoint(2);
            [plusPoint, minusPoint] = Line.get_parallel_point_from_distance(x, y, angle, distance);
        end
        
        function [dy2x, dy1x] = get_2nd_derivative_path(obj, startIndex, endIndex)
            if endIndex < startIndex+2
                error("endIndex not valid");
            end
            maxPathSize = size(obj.pathPoints,1);
            startIndex = min(startIndex, maxPathSize);
            endIndex = min(endIndex, maxPathSize);
            midIndexDiff = ceil((endIndex-startIndex)/2);
            %{
            fprintf("startindex: %d, midIndexdiff: %d, endindex: %d, sizemaxpp: %d\n", startIndex, ...
                midIndexDiff,endIndex, size(obj.pathPoints,1));
            %}
            point1 = obj.pathPoints(startIndex,:);
            point2 = obj.pathPoints(startIndex+midIndexDiff,:);
            point3 = obj.pathPoints(endIndex,:);
            dx = point2(1)-point1(1);
            dy1 = point2(2)-point1(2);
            dy2 = point3(2)-(2*point2(2))+point1(2);
            dy1x = dy1/dx;
            dy2x = dy2/dx;
            %{
            disp("2nd deriv & points")
            disp(point1)
            disp(point2)
            disp(point3)
            %}
            %disp("2nd deriv dy segment");
            %fprintf("dx:%.2f, dy1: %.2f, dy2: %.2f, dy1x: %.2f, dy2x: %.2f\n",dx, dy1, dy2, dy1x, dy2x);
        end
        function [dy2x, dy1x] = get_avg_2nd_derivative_path(obj, startIndex, endIndex)
            dy2x = 0;
            dy1x = 0;
            %segmentNum = endIndex-startIndex;
            %segmentLength = ceil(segmentNum/10);
            segmentNum = min(10, endIndex-startIndex);
            segmentLength = max(2,floor((endIndex-startIndex)/10));
            %fprintf("end-start: %.2f, segment num: %.2f, segment length: %.2f\n", (endIndex-startIndex), segmentNum, segmentLength);
            if segmentNum > 1
                secondDerivatives = zeros(segmentNum,1);
                for i = 1:size(secondDerivatives,1)
                    startI = startIndex+(i-1)*segmentLength;
                    endI = startIndex+i*segmentLength;
                    %endI = startI+2;
                    %fprintf("checking 2nd dev, startI: %d, endI:%d\n", startI, endI);
                    [dy2x, dy1x] = obj.get_2nd_derivative_path(startI, endI);
                    secondDerivatives(i) = dy2x;
                    %fprintf("dy2x: %.2f\n", dy2x);
                end
                %disp("2nd derivatives")
                %disp(secondDerivatives);
                dy2x = mean(secondDerivatives);
                dy1x = 0;
            end
        end
        
    end
    methods(Static)
        function distance = compare_distance(x1, y1, x2, y2)
            distance = sqrt((x1-x2)^2+(y1-y2)^2);
        end
    end
end