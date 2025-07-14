classdef AByPull < ABy
    properties
        mode = 'pull';
        numLegs = 2;
        wheelsA;
        wheelsBy;
        % variables
        actuated_variables = [sym('dot_waist_', [1 2]) sym('dot_hip_', [1 4]) sym('dot_knee_', [1 2])];
        unknown_variables = [sym('dot_alpha_', [1 2]) sym('dot_beta_', [1 2]) sym('dot_gamma_', [1 2])];
        desired_variables = [sym('dot_x_', [1 4]) sym('dot_y_', [1 4]) sym('dot_z_', [1 4]) ...
            sym('dot_alpha_3') sym('dot_alpha_4') ...
            sym('dot_beta_3') sym('dot_beta_4') ...
            sym('dot_gamma_3') sym('dot_gamma_4') ...
            sym('dot_steering_3') sym('dot_steering_4')];
        known_variables = [sym('dot_x_base') sym('dot_y_base') sym('dot_z_base') ...
            sym('dot_alpha_base') sym('dot_beta_base') sym('dot_gamma_base')];
    end
    methods
        %% Class Instance Functions
        function obj = AByPull(transformationDirectory)
            obj@ABy(transformationDirectory);
            
            temp = SaveTransformation.load_data(DtBB, obj.mode, obj.transformationDirectory);
            obj.equations = [temp.dot_x_base(1:2); temp.dot_y_base(1:2); temp.dot_z_base; ...
                temp.dot_alpha_base(1:2); temp.dot_beta_base(1:2); temp.dot_gamma_base(1:2)];
            
            % get coefficients
            obj.actuatedCoeffs = equationsToMatrix(obj.equations, obj.actuated_variables);
            obj.actuatedCoeffs = -obj.actuatedCoeffs;
            obj.unknownCoeffs = equationsToMatrix(obj.equations, obj.unknown_variables);
            obj.unknownCoeffs = -obj.unknownCoeffs;
            obj.desiredCoeffs = equationsToMatrix(obj.equations, obj.desired_variables);
            % obj.knownCoeffs = (rows[x1 x2 y1 y2 z1 z2 z3 z4 a1 a2 b1 b2 g1 g2], cols[x y z a b g])
            I6 = eye(6);
            obj.knownCoeffs = [repmat(I6(1,:),2,1); repmat(I6(2,:),2,1); repmat(I6(3,:),4,1); ...
                repmat(I6(4,:),2,1); repmat(I6(5,:),2,1); repmat(I6(6,:),2,1)];
            obj.knownCoeffs = -obj.knownCoeffs;
            
            obj = obj.save_aby();
            % legs 1 and 2 only
            walkA = obj.A([1:6 9:end],[1:4 7:end]);
            walkB = obj.B([1:6 9:end], [1:2 5:6 9:10 21:end]);
            walky = obj.y([1:2 5:6 9:10 21:end]);
            walkBy = simplify(walkB*walky);
            obj.partition_legs(obj.numLegs, walkA, walkBy);
            % wheels 3 and 4
            obj = obj.partition_wheels();
        end
        % include wheels in partition
        function obj = partition_wheels(obj)    
            % wheels -> same for all phases
            % rows(A) -> 5: dot_z_3, 6: dot_z_4
            % cols(A),cols(By) -> 5: dot_hip_3, 6: dot_hip_4
            A = obj.A;
            By = obj.By;
            wheelsA = A(7:8, 5:6);
            wheelsBy = By(7:8,:);
            
            fileName = obj.get_filename(obj.mode);
            save(fileName, 'wheelsA', 'wheelsBy', '-append');
        end
    end
end