classdef SinBumpyP8HighTerrainArgs < TerrainArgsInterface
    methods
        function obj = SinBumpyP8HighTerrainArgs()
            %% terrain settings
            obj.terrainName = 'sin';
            obj.genTerrainFromPath = true;
            % custom, ignore if gen from path
            %obj.width = pathEndX+100;
            %obj.height = pathEndY+200;
            obj.cellsize = 5;
            % sin
            obj.terrainAmplitude = 13;
            obj.terrainFrequency = 0.8;
            % random
            obj.randFilterSize = 10;
            obj.elevationChangeRange = 14;
            %terrainName = 'customTerrainName'; 
        end
    end
end