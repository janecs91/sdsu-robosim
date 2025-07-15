%% Symbolic Transformation matrices
classdef Transformation < SaveTransformation
    properties
        dhType = 'modified';
        dhTable;
        %% Joint Names
        % hip, eta
        % waist, omega
        % knee, nu
        % ankle, epsilon
        % contact, kappa
        % steering, psi
    end
    properties(Constant)
        %% Save File Properties
        savePrefix = 'tbe_';
        %% Constant Values
        % a_0 = d_0 = 15.5 cm (half-width robot body)
        % a_2 = 6 cm (distance robot body to first joint)
        % a_3 = a_4 = 24 cm (leg length)
        % d_1 = 15 cm (half-height robot body)
        % d_5 = 7 cm (distance between ankle to wheel axel) --> 6?
        % d_7 = 6 cm (wheel radius)
        constantKeys = {'a_0', 'a_2', 'a_3', 'a_4', 'd_0', 'd_1', 'd_5', 'd_7'}
        constantValues = {15.5 6 24 24 15.5 15 6 6}
        constants = containers.Map(Transformation.constantKeys, Transformation.constantValues)
    end
    methods
        %% Class Instance Functions
        function obj = Transformation(transformationDirectory, dhTable)
            if nargin < 1
                transformationDirectory = '';
            end
            obj@SaveTransformation(transformationDirectory);
            if nargin > 1
                obj.dhTable = dhTable;
            end
            
            if exist('dhTable', 'var')
                % sub constants here? --> sequence doesn't match for silo
                % TBD: test different sequences later
                obj.dhTable = Utils.sub_values_from_map(dhTable, obj.constants);
            end
        end
        function obj = save(obj)
            if ~isempty(obj.dhTable)
                % calculate and save transformation matrices per leg
                obj = obj.save_transformation_matrix(obj.dhTable, obj.dhType);
            end
        end
        function obj = save_transformation_matrix(obj, dhTable, dhType)
            fprintf('Saving transformation matrix for %s\n', obj.mode);
            legs = 4;
            tBE = cell(4,1);
            x_leg = sym(zeros(4,1));
            y_leg = sym(zeros(4,1));
            z_leg = sym(zeros(4,1));
            alpha_leg = sym(zeros(4,1));
            beta_leg = sym(zeros(4,1));
            gamma_leg = sym(zeros(4,1));
            rot_matrix_leg = cell(4,1);
            accumulated_matrices = cell(4,1);
            next_matrices = cell(4,1);
            if size(dhTable) < legs
                % fail
                error('dhTable missing for some legs.');
            end
            % loop through given legs
            for leg = 1:legs
                [tBE_leg, accumulatedMatrices, nextMatrices] = obj.calculate_tm_from_dh(dhTable{leg}, dhType);
                
                % sub constants here --> matches silo
                %tBE_leg = Utils.sub_values_from_map(tBE_leg, obj.constants);
                % sub mode constants, if any exist
                if isprop(obj, 'modeConstants')
                    tBE_leg = Utils.sub_values_from_map(tBE_leg, obj.modeConstants);
                end
                tBE_leg = simplify(tBE_leg);
                tBE(leg) = {tBE_leg};
                
                % extract values
                x_leg(leg) = tBE_leg(1,4);
                y_leg(leg) = tBE_leg(2,4);
                z_leg(leg) = tBE_leg(3,4);
                % not sure if skew symmetric?? 
                %(alp, bet, gam -> probs wrong)
                %alpha_leg(leg) = tBE_leg(3,2);
                %beta_leg(leg) = tBE_leg(1,3);
                %gamma_leg(leg) = tBE_leg(2,1); 
                rot_matrix_leg(leg) = {tBE_leg(1:3,1:3)};
                
                % save accumulated matrices for visualization/animation
                accumulated_matrices(leg) = {accumulatedMatrices};
                next_matrices(leg) = {nextMatrices};
            end

            fileName = obj.get_filename(obj.mode);
            save(fileName, 'tBE', 'x_leg', 'y_leg', 'z_leg', ...
                'rot_matrix_leg', 'accumulated_matrices', 'next_matrices');
        end
        function save_dtbb(obj)
            DtBB(obj.mode, obj.transformationDirectory);
        end
    end
    methods(Static)
        %% Static Functions
        function dhTable = get_dh()
            dhTable = obj.dhTable;
        end
        function [transformationMatrix, accumulatedMatrices, nextMatrices] = calculate_tm_from_dh(dhTable, dhType)
            accumulatedMatrices = cell(size(dhTable,1),1);
            nextMatrices = cell(size(dhTable,1),1);
            i = 1;
            transformationMatrix = eye(4);
            switch dhType
                case 'modified'
                    % modified DH
                    for row = dhTable.'
                        nextMatrix = Transformation.get_dh_matrix_modified(row(1), row(2), row(3), row(4));
                        transformationMatrix = transformationMatrix*nextMatrix;
                        transformationMatrix = simplify(transformationMatrix);
                        accumulatedMatrices(i) = {transformationMatrix};
                        nextMatrices(i) = {nextMatrix};
                        i = i+1;
                    end
                otherwise
                    % classic DH (default)
                    for row = dhTable.'
                        nextMatrix = Transformation.get_dh_matrix_classic(row(1), row(2), row(3), row(4));
                        transformationMatrix = transformationMatrix*nextMatrix;
                        accumulatedMatrices(i) = {transformationMatrix};
                        nextMatrices(i) = {nextMatrix};
                        i = i+1;
                    end
            end
            %transformationMatrix = simplify(transformationMatrix);
        end
        function dhMatrix = get_dh_matrix_classic(theta, d, alpha, r)
            dhMatrix = [cos(theta) -sin(theta)*cos(alpha) sin(theta) r*cos(theta);
                sin(theta) cos(theta)*cos(alpha) -cos(theta)*sin(alpha) r*cos(theta);
                0 sin(alpha) cos(alpha) d;
                0 0 0 1];
        end
        function dhMatrix = get_dh_matrix_modified(theta, d, alpha, r)
            dhMatrix = [cos(theta) -sin(theta) 0 r;
                cos(alpha)*sin(theta) cos(theta)*cos(alpha) -sin(alpha) -d*sin(alpha);
                sin(theta)*sin(alpha) sin(alpha)*cos(theta) cos(alpha) d*cos(alpha);
                0 0 0 1];
        end
        function rotationMatrix = get_rotation_matrix(alpha, beta, gamma)
            rotX = [1 0 0; 0 cos(alpha) -sin(alpha); 0 sin(alpha) cos(alpha)];
            rotY = [cos(beta) 0 sin(beta); 0 1 0; -sin(beta) 0 cos(beta)];
            rotZ = [cos(gamma) -sin(gamma) 0; sin(gamma) cos(gamma) 0; 0 0 1];
            rotationMatrix = rotZ*rotY*rotX;
        end
    end
end