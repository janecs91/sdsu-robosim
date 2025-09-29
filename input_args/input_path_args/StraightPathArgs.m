classdef StraightPathArgs < PathArgsInterface
    methods
        function obj = StraightPathArgs()
            %% path settings
            obj.pathName = 'straight';
            obj.pathStartX = 100; 
            obj.pathStartY = 100;
            obj.pathEndX = 2100;
            obj.pathEndY = pathStartY + 100;
            obj.pathAmplitude = 100;
        end
    end
end