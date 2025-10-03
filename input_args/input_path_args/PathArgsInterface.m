classdef PathArgsInterface
    properties(Constant)
        pathOptions = {'straight', 'line', 'halfcirc', 'halfcirc2', 'sin'};
    end
    properties
        pathName
        pathStartX
        pathStartY
        pathEndX
        pathEndY
        pathAmplitude
    end
    methods
        function obj = PathArgsInterface()
            %% path settings
            % 1, 2, 3, 5
            %obj.pathNum = pathNum;
            %{
            obj.pathName = pathName;
            obj.pathStartX = pathStartX; 
            obj.pathStartY = pathStartY;
            obj.pathEndX = pathEndX;
            obj.pathEndY = pathEndY;
            obj.pathAmplitude = pathAmplitude;
            %}
        end

        function pathArgs = getPathArgs(obj)
            pathRequiredArgs = {obj.pathName, obj.pathStartX, obj.pathStartY, obj.pathEndX, obj.pathEndY};
            pathExtraArgs = {obj.pathAmplitude};
            pathArgs = [pathRequiredArgs pathExtraArgs];
        end
    end
end