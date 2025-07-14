classdef InvAByPull < InvABy
    properties
        mode = 'pull'
        numLegs = 2
    end
    methods
        function obj = InvAByPull(transformationDirectory)
            obj@InvABy(transformationDirectory);
            obj.save_invaby();
        end
        % include wheels
        function save_invaby(obj)
            % use walk mode invaby()
            InvAByWalk.save_invaby(obj.mode, obj.numLegs)
            
            % wheels
            % load wheelsA, wheelsBy
            temp = SaveTransformation.load_data(ABy, obj.mode, obj.transformationDirectory);
            wheelsA = temp.wheelsA;
            wheelsBy = temp.wheelsBy;
            % calculate inv*by for wheels
            invAWheels = pinv(wheelsA);
            invAWheels = simplify(invAWheels);
            vectorXWheels = invAWheels*wheelsBy;
            vectorXWheels = simplify(vectorXWheels);
            
            fileName = obj.get_filename(obj.mode);
            save(fileName, 'invAWheels', 'vectorXWheels', '-append');
        end
    end
end