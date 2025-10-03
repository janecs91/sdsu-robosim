classdef DiagonalPathArgs < PathArgsInterface
    methods
        function obj = DiagonalPathArgs()
            %% path settings
            obj.pathName = 'line';
            obj.pathStartX = 100; 
            obj.pathStartY = 100;
            obj.pathEndX = 2100;
            obj.pathEndY = obj.pathStartY + 2000;
            obj.pathAmplitude = 100;
        end
    end
end