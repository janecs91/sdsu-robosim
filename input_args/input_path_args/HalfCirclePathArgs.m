classdef HalfCirclePathArgs < PathArgsInterface
    methods
        function obj = HalfCirclePathArgs()
            %% path settings
            obj.pathName = 'halfcirc';
            obj.pathStartX = 100; 
            obj.pathStartY = 100;
            obj.pathEndX = 2100;
            obj.pathEndY = pathStartY + 1000;
            obj.pathAmplitude = 100;
        end
    end
end