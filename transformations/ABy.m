%% Symbolic A, By matrices/vectors
classdef ABy < SaveTransformation
    properties
        %mode;
        %{
        actuated_variables;
        unknown_variables;
        desired_variables;
        known_variables;
        %}
        
        equations;
        actuatedCoeffs;
        unknownCoeffs;
        desiredCoeffs;   
        knownCoeffs;
        A;
        x;
        B;
        y;
        By;
    end
    properties(Constant)
        %% Save File Properties
        savePrefix = 'aby_';
    end
    methods
        function obj = ABy(transformationDirectory)
            if nargin < 1
                transformationDirectory = '';
            end
            obj@SaveTransformation(transformationDirectory);
        end
        %% Class Instance Functions
        function obj = save_aby(obj)
            fprintf('Saving A, By matrix/vector for %s\n', obj.mode);
            % Ax = By
            % A[actuated unknown] = B[desired known]
            % [actuated unknown] = A^-1 * B[desired known]
            % x = (A^-1)(By)

            % get A, B matrices
            % get y vector
            A = [obj.actuatedCoeffs obj.unknownCoeffs];
            x = [obj.actuated_variables obj.unknown_variables];
            B = [obj.desiredCoeffs obj.knownCoeffs];
            y = [obj.desired_variables obj.known_variables].';
            By = simplify(B*y);
            obj.A = A;
            obj.x = x;
            obj.B = B;
            obj.y = y;
            obj.By = By;
            
            fileName = sprintf(obj.savePath, obj.transformationDirectory, obj.savePrefix, obj.mode);
            save(fileName, 'A', 'B', 'y', 'By', 'x');
        end
        % partition A,B,y into legs/body
        function partition_legs(obj, numLegs, A, By)
            ALeg = cell(numLegs,1);
            ByLeg = cell(numLegs,1);
            ABody = cell(numLegs,1);
            ByBody = cell(numLegs,1);

            for activeLeg = 1:numLegs
                % leg
                ALeg(activeLeg) = {A(activeLeg:numLegs:end, activeLeg:numLegs:end)};
                ByLeg(activeLeg) = {By(activeLeg:numLegs:end, :)};
                
                % body
                leftoverA = A;
                leftoverA(activeLeg:numLegs:end, :) = [];
                leftoverA(:, activeLeg:numLegs:end) = [];
                leftoverBy = By;
                leftoverBy(activeLeg:numLegs:end, :) = [];
                ABody(activeLeg) = {leftoverA};
                ByBody(activeLeg) = {leftoverBy};
            end
            
            fileName = obj.get_filename(obj.mode);
            save(fileName, 'ALeg', 'ByLeg', 'ABody', 'ByBody', '-append');
        end
    end
end