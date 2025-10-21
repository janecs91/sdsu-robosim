classdef SinEasyAmp12TerrainArgs < TerrainArgsInterface
    methods
        function obj = SinEasyAmp12TerrainArgs()
            %% terrain settings
            obj.terrainName = 'sin';
            obj.genTerrainFromPath = true;
            % custom, ignore if gen from path
            %obj.width = pathEndX+100;
            %obj.height = pathEndY+200;
            obj.cellsize = 5;
            % sin
            obj.terrainAmplitude = 12;
            obj.terrainFrequency = 0.01;
            % random
            obj.randFilterSize = 10;
            obj.elevationChangeRange = 14;
            %terrainName = 'customTerrainName'; 
        end
    end
end