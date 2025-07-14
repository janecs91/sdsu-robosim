classdef Random < TerrainGenerator
    properties(Constant)
        % vars: width, height, filter size, elevation range
        name = 'random_%.0d_%.0d_%.0d_%.0d';
        argNames = {'elevationChangeRange', 'randFilterSize'};
    end
    methods
        function obj = Random(directory)
            obj@TerrainGenerator(directory);
        end
        function fileAddress = generate(obj, xRange, yRange, scale, filterSize, elevationChangeRange)
            if nargin < 4
                filterSize = 5;
            end
            if nargin < 5
                elevationChangeRange = 1;
            end
            disp("RAND ARGS")
            disp(nargin)
            disp(xRange)
            disp(yRange)
            disp(scale)
            disp(filterSize)
            disp(elevationChangeRange)
            initX = xRange(1);
            initY = yRange(1);
            scaleX = scale;
            scaleY = scale;
            maxX = xRange(2);
            maxY = yRange(2);
            lengthX = length(initX:scaleX:maxX);
            lengthY = length(initY:scaleY:maxY);
            
            elevationMatrix = zeros(lengthX, lengthY);
            elevationMatrix(1,:) = randn(1, lengthY);
            xIndex = 2;
            for x = 1:lengthX-1
                randomizedZValue = randn(1, lengthY);
                randomizedZSign = randn(1, lengthY);
                randomizedZSign(randomizedZSign < 0.5) = -1;
                randomizedZSign(randomizedZSign >= 0.5) = 1;
                elevationsAcrossY = elevationMatrix(xIndex-1,:)+randomizedZValue.*randomizedZSign.*elevationChangeRange;
                elevationMatrix(xIndex,:) = elevationsAcrossY;
                xIndex = xIndex + 1;
            end
            
            % smooth elevations?
            K = (1/(filterSize*filterSize))*ones(filterSize);
            elevationMatrix = conv2(elevationMatrix,K,'same');
            
            slipMatrix = zeros(size(elevationMatrix));
            
            % save
            terrainName = sprintf(obj.name, maxX, maxY, filterSize, elevationChangeRange);
            fileAddress = sprintf(obj.saveFileAddress,terrainName);
            save(fileAddress, 'initX', 'initY', 'maxX', 'maxY', 'scaleX', 'scaleY', ...
            'elevationMatrix', 'slipMatrix');
        end
    end
end