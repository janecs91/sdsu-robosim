classdef SinHighEasyTerrainArgs < TerrainArgsInterface
    methods
        function obj = SinHighEasyTerrainArgs()
            %% terrain settings
            obj.terrainName = 'sin';
            obj.genTerrainFromPath = true;
            % custom, ignore if gen from path
            %obj.width = pathEndX+100;
            %obj.height = pathEndY+200;
            obj.cellsize = 5;
            % sin
            obj.terrainAmplitude = 15;
            obj.terrainFrequency = 0.01;
            % random
            obj.randFilterSize = 10;
            obj.elevationChangeRange = 14;
            %terrainName = 'customTerrainName'; 
        end
    end
end