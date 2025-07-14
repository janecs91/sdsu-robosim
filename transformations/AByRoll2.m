classdef AByRoll2 < ABy
    properties
        mode = 'roll';
        % variables
        actuated_variables = sym('dot_hip_', [1 4]);
        %actuated_variables = [sym('dot_hip_', [1 4]) sym('dot_steering_', [1 4])];
        unknown_variables = [sym('dot_z_base') sym('dot_alpha_base') sym('dot_beta_base')];
        desired_variables = [];
        
        known_variables = [sym('dot_x_', [1 4]) sym('dot_y_', [1 4]) sym('dot_z_', [1 4]) ...
            sym('dot_alpha_', [1 4]) sym('dot_beta_', [1 4]) sym('dot_gamma_', [1 4]) ... 
            sym('dot_steering_', [1 4])];
    end
    methods
        %% Class Instance Functions
        function obj = AByRoll2(transformationDirectory)
            obj@ABy(transformationDirectory);
            
            temp = SaveTransformation.load_data(DtBB, obj.mode, obj.transformationDirectory);
            obj.equations = [temp.dot_z_base; temp.dot_alpha_base; temp.dot_beta_base];

            % get coefficients
            obj.actuatedCoeffs = equationsToMatrix(obj.equations, obj.actuated_variables);
            obj.actuatedCoeffs = -obj.actuatedCoeffs;
            I3 = eye(3);
            obj.unknownCoeffs = [repmat(I3(1,:),4,1); repmat(I3(2,:),4,1); repmat(I3(3,:),4,1);];
            obj.unknownCoeffs = -obj.unknownCoeffs;
            obj.knownCoeffs = equationsToMatrix(obj.equations, obj.known_variables);
            obj.desiredCoeffs = [];
            
            disp('dimensions');
            disp(obj.unknownCoeffs);
            disp(size(obj.actuatedCoeffs));
            disp(size(obj.unknownCoeffs));
            disp(size(obj.knownCoeffs));
            
            obj = obj.save_aby();
            obj = obj.save_as_wheels();
        end
        function obj = save_as_wheels(obj)
            disp('aby dim')
            disp(size(obj.A));
            disp(size(obj.By));
            disp(size(obj.unknown_variables));
            disp(size(obj.A(1:4,5)));
            
            
            wheelsA = obj.A(1:4,1:4);
            unknownsExtended = [repmat([obj.unknown_variables(1) 0 0],4,1); ...
                repmat([0 obj.unknown_variables(2) 0],4,1); ...
                repmat([0 0 obj.unknown_variables(3)],4,1)];
            disp(unknownsExtended(1:4,1));
            wheelsBy = obj.By(1:4) - unknownsExtended(1:4,1).*obj.A(1:4,5);
            
            
            fileName = obj.get_filename(obj.mode);
            save(fileName, 'wheelsA', 'wheelsBy', '-append');
        end
    end
end