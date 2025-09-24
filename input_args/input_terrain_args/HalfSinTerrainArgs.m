classdef HalfSinTerrainArgs < TerrainArgsInterface
    methods
        function obj = HalfSinTerrainArgs()
            %% terrain settings
            obj.terrainName = 'halfsin';
            obj.genTerrainFromPath = true;
            % custom, ignore if gen from path
            %obj.width = pathEndX+100;
            %obj.height = pathEndY+200;
            obj.cellsize = 5;
            % sin
            obj.terrainAmplitude = 13;
            obj.terrainFrequency = 1;
            % random
            obj.randFilterSize = 10;
            obj.elevationChangeRange = 14;
            %terrainName = 'customTerrainName'; 
        end
    end
end