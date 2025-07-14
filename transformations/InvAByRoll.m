classdef InvAByRoll < InvABy
    properties
        mode = 'roll'
    end
    methods
        function obj = InvAByRoll(transformationDirectory)
            obj@InvABy(transformationDirectory);
            obj.save_invaby();
        end
        function save_invaby(obj)
            temp = SaveTransformation.load_data(ABy, obj.mode, obj.transformationDirectory);
            A = temp.A;
            By = temp.By;
            By = By - sym('dot_z_base')*A(:,5);
            invA = pinv(A(:,1:4));
            invA = simplify(invA);
            vectorX = invA*By;
            vectorX = simplify(vectorX);
            fileName = sprintf(obj.save_path, obj.save_prefix, obj.mode);
            save(fileName, 'invA', 'vectorX');
        end
    end
end