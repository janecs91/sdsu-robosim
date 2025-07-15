classdef AByWalk < ABy
    properties
        mode = 'walk';
        numLegs = 4;
        % variables
        actuated_variables = [sym('dot_waist_', [1 4]) sym('dot_hip_', [1 4]) sym('dot_knee_', [1 4])];
        unknown_variables = [sym('dot_alpha_', [1 4]) sym('dot_beta_', [1 4]) sym('dot_gamma_', [1 4])];
        desired_variables = [sym('dot_x_', [1 4]) sym('dot_y_', [1 4]) sym('dot_z_', [1 4])];
        known_variables = [sym('dot_x_base') sym('dot_y_base') sym('dot_z_base') ...
            sym('dot_alpha_base') sym('dot_beta_base') sym('dot_gamma_base')];
    end
    methods
        %% Class Instance Functions
        function obj = AByWalk(transformationDirectory)
            obj@ABy(transformationDirectory);
            
            temp = SaveTransformation.load_data(DtBB, obj.mode, obj.transformationDirectory);
            obj.equations = [temp.dot_x_base; temp.dot_y_base; temp.dot_z_base; ...
                temp.dot_alpha_base; temp.dot_beta_base; temp.dot_gamma_base];
            
            % get coefficients
            obj.actuatedCoeffs = equationsToMatrix(obj.equations, obj.actuated_variables);
            obj.actuatedCoeffs = -obj.actuatedCoeffs;
            obj.unknownCoeffs = equationsToMatrix(obj.equations, obj.unknown_variables);
            obj.unknownCoeffs = -obj.unknownCoeffs;
            obj.desiredCoeffs = equationsToMatrix(obj.equations, obj.desired_variables);
            I6 = eye(6);
            obj.knownCoeffs = [repmat(I6(1,:),4,1); repmat(I6(2,:),4,1); repmat(I6(3,:),4,1); ...
                repmat(I6(4,:),4,1); repmat(I6(5,:),4,1); repmat(I6(6,:),4,1)]; 
            obj.knownCoeffs = -obj.knownCoeffs;
            
            obj = obj.save_aby();
            obj.partition_legs(obj.numLegs, obj.A, obj.By);
            
        end
    end
    methods(Static)
        
    end
end