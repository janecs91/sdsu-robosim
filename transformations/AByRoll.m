classdef AByRoll < ABy
    properties
        mode = 'roll';
        % variables
        actuated_variables = sym('dot_hip_', [1 4]);
        %actuated_variables = [sym('dot_hip_', [1 4]) sym('dot_steering_', [1 4])];
        unknown_variables = sym('dot_z_base');
        desired_variables = [];
        
        known_variables = [sym('dot_x_', [1 4]) sym('dot_y_', [1 4]) sym('dot_z_', [1 4]) ...
            sym('dot_alpha_', [1 4]) sym('dot_beta_', [1 4]) sym('dot_gamma_', [1 4]) ... 
            sym('dot_steering_', [1 4])];
        %{
        known_variables = [sym('dot_x_', [1 4]) sym('dot_y_', [1 4]) sym('dot_z_', [1 4]) ...
            sym('dot_alpha_', [1 4]) sym('dot_beta_', [1 4]) sym('dot_gamma_', [1 4])];
        %}
    end
    methods
        %% Class Instance Functions
        function obj = AByRoll(transformationDirectory)
            obj@ABy(transformationDirectory);
            
            temp = SaveTransformation.load_data(DtBB, obj.mode, obj.transformationDirectory);
            obj.equations = temp.dot_z_base;

            % get coefficients
            obj.actuatedCoeffs = equationsToMatrix(obj.equations, obj.actuated_variables);
            obj.actuatedCoeffs = -obj.actuatedCoeffs;
            obj.unknownCoeffs = ones(4,1);
            obj.knownCoeffs = equationsToMatrix(obj.equations, obj.known_variables);
            obj.desiredCoeffs = [];
            
            obj = obj.save_aby();
            obj = obj.save_as_wheels();
        end
        function obj = save_as_wheels(obj)
            wheelsA = obj.A(:,1:4);
            wheelsBy = obj.By - obj.unknown_variables*obj.A(:,5);
            
            fileName = obj.get_filename(obj.mode);
            save(fileName, 'wheelsA', 'wheelsBy', '-append');
        end
    end
end