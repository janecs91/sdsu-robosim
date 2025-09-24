classdef SinPathArgs < PathArgsInterface
    methods
        function obj = SinPathArgs()
            %% path settings
            % 1, 2, 3, 5
            %{
            obj.pathNum = 2;
            obj.pathStartX = 100; 
            obj.pathStartY = 100;
            obj.pathEndX = 2150;
            obj.pathEndY = pathStartY + 1000;
            obj.pathAmplitude = 100;
            %}
            obj@PathArgsInterface('sin', 100, 100, 2150, 2150+1000, 100)
        end
    end
end