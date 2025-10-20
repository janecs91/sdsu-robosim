classdef MatrixTerrain
    properties
        fileAddress;
        terrainName;
        % xy matrix of elevation z
        elevationMatrix;
        % terrain size, saved for convenience
        gridSize;
        % scale of each grid square in cm?
        slipMatrix;
        sinkRate;
        % ranges
        initX;
        initY;
        scaleX;
        scaleY;
        maxX;
        maxY;
    end
    methods
        function obj = MatrixTerrain(fileAddress, directory, loadFromSavedTerrain, terrainArgs)
            if nargin > 0
                terrainAddress = sprintf('%s/%s.mat\n', directory, fileAddress);
                fprintf('attempting to load terrain address %s\n', terrainAddress);
            else
                terrainAddress = 'terrain_flat_1060.mat';
            end
            if nargin < 3
                terrainArgs = {};
            end
            % load terrain
            addpath('terrain/generators')
            terrainGenerator = TerrainGenerator(directory);
            try
                if ~(loadFromSavedTerrain)
                   error("Ignoring load from save...")
                end
                terrainData = load(terrainAddress);
                fprintf("Successfully loaded terrain: %s\n", terrainAddress)
            catch
                if loadFromSavedTerrain
                    fprintf('error: could not find terrain %s\n', terrainAddress)
                end
                disp('generating...');
                % generate if doesn't exist
                if nargin > 2
                    disp("options gen")
                    terrainAddress = terrainGenerator.gen_from_options(terrainArgs{:});
                else
                    disp("string gen")
                    terrainAddress = terrainGenerator.gen_from_string(terrainAddress);
                end
                terrainData = load(terrainAddress);
            end
            obj.fileAddress = terrainAddress;
            obj.terrainName = terrainGenerator.get_name(terrainArgs{:});
            
            obj.elevationMatrix = terrainData.elevationMatrix;
            obj.slipMatrix = terrainData.slipMatrix;
            obj.sinkRate = 0;
            obj.gridSize = size(obj.elevationMatrix);
            obj.scaleX = terrainData.scaleX;
            obj.scaleY = terrainData.scaleY;
            obj.initX = terrainData.initX;
            obj.initY = terrainData.initY;
            obj.maxX = terrainData.maxX;
            obj.maxY = terrainData.maxY;
        end
        function slipRates = get_slip_rates(obj, xVector, yVector)
            indices = obj.get_index(xVector, yVector);
            slipRates = squeeze(obj.slipMatrix(indices));
        end
        function elevations = get_elevations(obj, xVector, yVector)
            elevations = zeros(length(xVector),1);
            for i=1:length(elevations)
                elevations(i, 1) = obj.get_elevation(xVector(i), yVector(i));
            end
        end
        function elevation = get_elevation(obj, x, y)
            if length(x) > 1 || length(y) > 1
                error('Too many x,y coordinates given. Use get_elevations() instead')
            end
            nearestInds = obj.get_nearest_indices(x,y);
            xVector = (nearestInds(:,1)-1).*obj.scaleX+obj.initX;
            yVector = (nearestInds(:,2)-1).*obj.scaleY+obj.initY;
            szElev = size(obj.elevationMatrix());
            zVector = obj.elevationMatrix(sub2ind(szElev,nearestInds(:,1),nearestInds(:,2)));
            coords = [xVector yVector zVector];
            if length(zVector) == 1
                % assume point
                elevation = zVector;
            elseif length(zVector) == 2
                if coords(1,1) == coords(2,1)
                    % x are same --> along y-axis
                    axis = 2;
                    lineX = y;
                else
                    % y are same --> along x-axis
                    axis = 1;
                    lineX = x;
                end
                [slope, intercept] = obj.get_line(coords(1,[axis 3]), coords(2,[axis 3]));
                elevation = slope*lineX + intercept;
            else
                % 3 indices
                [a, b, c, d] = obj.get_plane(coords(1,:), coords(2,:), coords(3,:));
                elevation = (d - a*x - b*y)/c;
            end
        end
        function gridIndex = get_index(obj, xVector, yVector)
            % input scalar or vector x,y
            % get closest grid index to x,y
            % assume floor
            gridSize = obj.gridSize;
            gridX = round(((xVector-obj.initX)/obj.scaleX)+1,8);
            gridY = round(((yVector-obj.initY)/obj.scaleY)+1,8);
            gridIndex = sub2ind(gridSize,max(1,min(gridSize(1),floor(gridX))), ...
                max(1,min(gridSize(2),floor(gridY))));
        end
        function gridIndices = get_nearest_indices(obj, x, y)
            % input only scalar x,y
            % get nearest grid indices to x,y
            % returns up to 3 indices
            gridSize = obj.gridSize;
            gridX = round(((x-obj.initX)/obj.scaleX)+1,8);
            gridY = round(((y-obj.initY)/obj.scaleY)+1,8);
            floorInd = [max(1,min(gridSize(1),floor(gridX))) max(1,min(gridSize(2),floor(gridY)))];
            ceilInd = [max(1,min(gridSize(1),ceil(gridX))) max(1,min(gridSize(2),ceil(gridY)))];
            if isequal(floorInd, ceilInd)
                % shares both x,y -> at a point
                gridIndices = floorInd;
            elseif floorInd(1) == ceilInd(1) || floorInd(2) == ceilInd(2)
                % shares x or y -> on a line
                gridIndices = [floorInd; ceilInd];
            else
                % all different -> on a plane
                altInds = [floorInd(1) ceilInd(2); ceilInd(1) floorInd(2)];
                searchResults = dsearchn([gridX gridY], altInds);
                nearestAltInd = altInds(searchResults(1),:);
                gridIndices = [floorInd; ceilInd; nearestAltInd];
            end
        end
        function highestElevation = get_highest_elevation(obj, x1, y1, x2, y2, scale)
           xVector = x1:scale:x2;
           yVector = linspace(y1, y2, size(xVector, 2));
           elevations = obj.get_elevations(xVector, yVector);
           highestElevation = max(elevations);
        end
        function angles = get_angle_elevations(obj, x1, y1, x2, y2)
            %disp('ANGLE ELEVATIONS')
            z1 = obj.get_elevations(x1, y1);
            z2 = obj.get_elevations(x2, y2);
            distances = sqrt((x1-x2).^2+(y1-y2).^2);
            angles = atan2(z2-z1, distances);
            %{
            disp(x1);
            disp(x2);
            disp(y1);
            disp(y2);
            disp(distances);
            disp(angles);
            %}
        end
    end
    methods(Static)
        function [slope, intercept] = get_line(coord1, coord2)
            % 2D line where y = z-value (elevation)
            % axis determines along x or y
            % mx + b = y
            slope = (coord1(2) - coord2(2))/(coord1(1) - coord2(1));
            intercept = coord1(2) - slope*coord1(1);
        end
        function [a, b, c, d] = get_plane(coord1, coord2, coord3)
            % ax + by + cz = d
            normalVector = cross(coord1 - coord2, coord1 - coord3);
            a = normalVector(1);
            b = normalVector(2);
            c = normalVector(3);
            d = sum(normalVector.*coord1);
        end
    end
end