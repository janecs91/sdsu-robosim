%% Symbolic DtBB matrices
classdef DtBB < SaveTransformation
    properties
        mode;
    end
    properties(Constant)
        %% Save File Properties
        savePrefix = 'dtbb_';
    end
    methods
        %% Class Instance Functions
        function obj = DtBB(mode, transformationDirectory)
            if nargin < 2
                transformationDirectory = '';
            end
            obj@SaveTransformation(transformationDirectory);
            if nargin > 1
                obj.mode = mode;
                % assume constants
                constants = keys(Transformation.constants);
                % check if file exists
                % load tBE data from file
                tmData = obj.load_data(Transformation, obj.mode, obj.transformationDirectory);
                tBE = tmData.tBE;
                obj.save_dtbb_from_tbe(tBE, constants);
            end
        end
        
        %% calculate and save dtBB matrix
        function save_dtbb_from_tbe(obj, givenTBE, constants)
            fprintf('Saving dtBB matrix for %s\n', obj.mode);
            legs = 4;
            % generate symbols
            x = sym('x_', [1 4]);
            y = sym('y_', [1 4]);
            z = sym('z_', [1 4]);
            alphas = sym('alpha_', [1 4]);
            betas = sym('beta_', [1 4]);
            gammas = sym('gamma_', [1 4]);

            % pre-allocate
            dtBB = cell(4,1);
            dot_x_base = sym(zeros(4,1));
            dot_y_base = sym(zeros(4,1));
            dot_z_base = sym(zeros(4,1));
            dot_alpha_base = sym(zeros(4,1));
            dot_beta_base = sym(zeros(4,1));
            dot_gamma_base = sym(zeros(4,1));

            % loop through given legs
            for leg = 1:legs
                x_i = x(leg);
                y_i = y(leg);
                z_i = z(leg);
                alpha_i = alphas(leg);
                beta_i = betas(leg);
                gamma_i = gammas(leg);

                % calculate transformation matrices
                % t = transformation matrix
                % dt = diff(t)
                % B = base
                % Ej = end
                % dtBB = tBEj * (dtEjEj*tEjB + dtEjB)
                tEjEj = [0 -gamma_i beta_i x_i; gamma_i 0 -alpha_i y_i; -beta_i alpha_i 0 z_i; 0 0 0 0];
                dtEjEj = DtBB.get_differential_tm(tEjEj, constants);
                tBEj = givenTBE{leg};
                tEjB = inv(tBEj);
                dtEjB = DtBB.get_differential_tm(tEjB, constants);
                dtBB_leg = simplify(expand(tBEj * (dtEjEj*tEjB + dtEjB)));
                dtBB(leg) = {dtBB_leg};

                % extract values
                dot_x_base(leg) = dtBB_leg(1,4);
                dot_y_base(leg) = dtBB_leg(2,4);
                dot_z_base(leg) = dtBB_leg(3,4);
                dot_alpha_base(leg) = dtBB_leg(3,2);
                dot_beta_base(leg) = dtBB_leg(1,3);
                dot_gamma_base(leg) = dtBB_leg(2,1);
            end

            fileName = obj.get_filename(obj.mode);
            save(fileName, 'dtBB', 'dot_x_base', 'dot_y_base', 'dot_z_base', ...
                'dot_alpha_base', 'dot_beta_base', 'dot_gamma_base');
        end
    end
    methods(Static)
        % possible rewrite
        % was c/ped from SILO
        function derivative = get_differential_tm(transformation, constants)
            symSet = symvar(transformation);
            derivative = zeros(size(transformation,1),size(transformation,2));
            if size(symSet)>0
                for i = 1:size(symSet,2)
                    derivative = diff(transformation,symSet(i)) * symDiff(symSet(i), constants) + derivative;
                end
                derivative = simplify(derivative);
            else
                derivative = 0;
            end

            % helper derivative function
            function derivative = symDiff(symValue, constants)
                if isa(symValue, 'sym') & not(ismember(constants, sprintf("%s", symValue)))
                    derivative = str2sym(sprintf('dot_%s',symValue));
                    %disp(symValue);
                    %disp(derivative);
                else 
                    derivative = 0; % derivative of constant is 0
                end
            end
        end
    end
end