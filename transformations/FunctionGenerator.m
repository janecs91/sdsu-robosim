%% Converts symbolic matrices into functions
classdef FunctionGenerator
    properties
        transformationDirectory;
        functionDirectory;
    end
    properties(Constant)
        savePath = '%s/%s%s.mat';
    end
    methods
        function obj = FunctionGenerator(transformationDirectory, functionDirectory)
            obj.transformationDirectory = transformationDirectory;
            obj.functionDirectory = functionDirectory;
        end
        function generate_function_aby(obj, mode)
            data = SaveTransformation.load_data(ABy, mode, obj.transformationDirectory);
            vars_position_legs = [sym('dot_x_', [1 4]), sym('dot_y_', [1 4]), sym('dot_z_', [1 4])];
            vars_orientation_legs = [sym('dot_alpha_', [1 4]), sym('dot_beta_', [1 4]), sym('dot_gamma_', [1 4])];
            var_position_base = [sym('dot_x_base'), sym('dot_y_base'), sym('dot_z_base')];
            var_orientation_base = [sym('dot_alpha_base'), sym('dot_beta_base'), sym('dot_gamma_base')];
            
            % if exist wheels (rolling + pulling)
            if isfield(data, 'wheelsA')
                fprintf('Generating functions A, By wheels for %s\n', mode);
                % wheels
                matlabFunction(data.wheelsA, data.wheelsBy, ...
                    'File', sprintf('%s/aby_%s_wheels', obj.functionDirectory, mode), ...
                    'Vars', {sym('hip_',[1 4]), sym('steering_',[1 4]), ...
                    reshape(vars_position_legs, [4 3]), reshape(vars_orientation_legs, [4 3]), ...
                    var_position_base, var_orientation_base});
            end

            % if exist legs (walking + pulling)
            if isfield(data, 'ALeg')
                fprintf('Generating functions A, By legs for %s\n', mode);
                for i=1:length(data.ALeg)
                    % leg
                    matlabFunction(data.ALeg{i}, data.ByLeg{i}, ...
                        'File', sprintf('%s/aby_%s_leg_%d', obj.functionDirectory, mode, i), ...
                        'Vars', {sprintf('waist_%i',i), sprintf('hip_%i',i), sprintf('knee_%i',i), ...
                        [sym(sprintf('dot_x_%i',i)) sym(sprintf('dot_y_%i',i)) sym(sprintf('dot_z_%i',i))], ...
                        [sym(sprintf('dot_alpha_%i',i)) sym(sprintf('dot_beta_%i',i)) sym(sprintf('dot_gamma_%i',i))], ...
                        [sym('dot_x_base') sym('dot_y_base') sym('dot_z_base')] ...
                        [sym('dot_alpha_base') sym('dot_beta_base') sym('dot_gamma_base')]});
                    % body
                    matlabFunction(data.ABody{i}, data.ByBody{i}, ...
                        'File', sprintf('%s/aby_%s_body_%d', obj.functionDirectory, mode, i), ...
                        'Vars', {sym('waist_',[1 4]), sym('hip_',[1 4]), sym('knee_',[1 4]), ...
                        reshape(vars_position_legs, [4 3]), reshape(vars_orientation_legs, [4 3]), ...
                        var_position_base, var_orientation_base});
                end
            end
        end
        function generate_function_rotation_matrix_leg(obj, mode, numLegs)
            fprintf('Generating functions rotation matrix leg for %s\n', mode);
            % get dot_x, dot_y, dot_z ??
            % logic from silo
            tBE = SaveTransformation.load_data(Transformation, mode, obj.transformationDirectory);
            for leg = 1:numLegs
                rotMatrix = tBE.rot_matrix_leg{leg};
                matlabFunction(rotMatrix, 'File', sprintf('%s/tbe_%s_rot_leg_%i', obj.functionDirectory, mode, leg), ...
                        'Vars', {sprintf('waist_%i',leg), sprintf('hip_%i',leg), sprintf('knee_%i',leg)});
            end
        end
        function generate_function_end_positions(obj, mode)
            fprintf('Generating function global end positions for %s\n', mode);
            % calc x,y,z end positions for all legs
            % get base position + rotationMatrix*[x,y,z]_leg
            syms alpha_base beta_base gamma_base
            rotMatrix = Transformation.get_rotation_matrix(alpha_base, beta_base, gamma_base);
            %rotMatrix = eye(3);     % update this later if alpha, beta, gamma changes
            tBE = SaveTransformation.load_data(Transformation, mode, obj.transformationDirectory);
            xyz = [tBE.x_leg tBE.y_leg tBE.z_leg]';
            rotxyz = rotMatrix*xyz;
            syms x_base y_base z_base
            positions = repmat([x_base y_base z_base], 4, 1) + rotxyz';
            matlabFunction(positions, 'File', sprintf('%s/tbe_%s_end_pos', obj.functionDirectory, mode), ...
                    'Vars', {sym('waist_',[1 4]), sym('hip_',[1 4]), sym('knee_',[1 4]), ...
                    [x_base y_base z_base], [alpha_base beta_base gamma_base]});
        end
        % transformation matrices for visualization
        % generates function to evaluate accumulated matrices
        % gets each joint x,y,z position
        function generate_function_joint_positions(obj, mode)
            fprintf('Generating function global joint positions for %s\n', mode);
            
            tBE = SaveTransformation.load_data(Transformation, mode, obj.transformationDirectory);
            positions = cell(4,1);
            syms alpha_base beta_base gamma_base
            rotMatrix = Transformation.get_rotation_matrix(alpha_base, beta_base, gamma_base);
            for i=1:4
                accum_matrix = tBE.accumulated_matrices{i};
                numJoints = length(accum_matrix);
                accum_matrix = vertcat(accum_matrix{:});
                leg_positions = accum_matrix(:,4);
                leg_positions(4:4:end) = [];
                % reformat leg positions
                disp('size lp rm');
                disp(size(leg_positions));
                disp(size(rotMatrix));
                leg_positions = reshape(leg_positions, [3 numJoints]);
                disp('size lp 2');
                disp(size(leg_positions));
                
                leg_positions = rotMatrix*leg_positions;
                disp('size lp 3');
                syms x_base y_base z_base
                disp(size(repmat([x_base y_base z_base], numJoints, 1)));
                
                leg_positions = repmat([x_base y_base z_base], numJoints, 1) + leg_positions.';
                positions(i) = {leg_positions};
            end
            
            matlabFunction(positions{:}, 'File', sprintf('%s/tbe_%s_joint_pos', obj.functionDirectory, mode), ...
                'Vars', {sym('waist_', [1 4]), sym('hip_', [1 4]), sym('knee_', [1 4]), sym('steering_', [1 4]) ...
                [x_base y_base z_base], [alpha_base beta_base gamma_base]});
        end
    end
end