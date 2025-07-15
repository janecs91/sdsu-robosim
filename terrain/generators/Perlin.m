classdef Perlin < TerrainGenerator
    properties(Constant)
        % vars: width, height
        name = 'perlin_%d_%d';
        argNames = {};
    end
    methods
        function obj = Perlin(directory)
            obj@TerrainGenerator(directory);
        end
        function fileAddress = generate(obj, xRange, yRange, scale)
            %period = (2*pi)*(1/frequency);
            initX = xRange(1);
            initY = yRange(1);
            scaleX = scale;
            scaleY = scale;
            maxX = xRange(2);
            maxY = yRange(2);
            lengthX = length(initX:scaleX:maxX);
            lengthY = length(initY:scaleY:maxY);
            
            %{
            n = 64;
            m = 64;
            im = zeros(n, m);
            im = perlin_noise(im);
            function im = perlin_noise(im)
                [n, m] = size(im);
                i = 0;
                w = sqrt(n*m);
                while w > 3
                    i = i + 1;
                    d = interp2(randn(n, m), i-1, 'spline');
                    im = im + i * d(1:n, 1:m);
                    w = w - ceil(w/2 - 1);
                end
            %}
            
            %[n, m] = size(im);
            n = lengthX;
            m = lengthY;
            %im = zeros(n, m);
            elevationMatrix = zeros(lengthX, lengthY);
            i = 0;
            w = sqrt(n*m);
            while w > 3
                i = i + 1;
                d = interp2(randn(n, m), i-1, 'spline');
                elevationMatrix = elevationMatrix + i * d(1:n, 1:m);
                w = w - ceil(w/2 - 1);
            end
                        
            slipMatrix = zeros(size(elevationMatrix));
            
            % save
            terrainName = sprintf(obj.name, maxX, maxY);
            fileAddress = sprintf(obj.saveFileAddress,terrainName);
            save(fileAddress, 'initX', 'initY', 'maxX', 'maxY', 'scaleX', 'scaleY', ...
            'elevationMatrix', 'slipMatrix');
        end
    end
end