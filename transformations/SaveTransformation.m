%% Save File Properties
classdef SaveTransformation
    properties(Abstract = true)
    end
    properties
        transformationDirectory;
    end
    properties(Constant)
        savePath = '%s/%s%s.mat';
    end
    methods
        function obj = SaveTransformation(transformationDirectory)
            disp(transformationDirectory);
            obj.transformationDirectory = transformationDirectory;
        end
        function fileName = get_filename(obj, identifier)
            % identifier = mode
            fileName = sprintf(obj.savePath, obj.transformationDirectory, obj.savePrefix, identifier);
        end
    end
    methods(Static)
        function data = load_data(transformationClass, mode, loadFromDir)
            fileName = sprintf(SaveTransformation.savePath, loadFromDir, transformationClass.savePrefix, mode);
            data = load(fileName);
        end
    end
end