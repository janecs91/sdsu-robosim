classdef InvAByWalk < InvABy
    properties
        mode = 'walk'
        numLegs = 4
    end
    properties(Constant)
        % substitute known values
        baseKeys = {'dot_alpha_base' 'dot_beta_base' 'dot_gamma_base'}
        baseValues = {0 0 0}
        baseConstants = containers.Map(InvAByWalk.baseKeys, InvAByWalk.baseValues)
        bodyKeys = {'dot_x_base' 'dot_y_base' 'dot_z_base'}
        bodyValues = {0 0 0}
        bodyConstants = containers.Map(InvAByWalk.bodyKeys, InvAByWalk.bodyValues)
        legKeys = {'dot_x_1', 'dot_x_2', 'dot_x_3', 'dot_x_4', ...
            'dot_y_1', 'dot_y_2', 'dot_y_3', 'dot_y_4', ...
            'dot_z_1', 'dot_z_2', 'dot_z_3', 'dot_z_4'}
        legValues = {0 0 0 0 0 0 0 0 0 0 0 0};
        legConstants = containers.Map(InvAByWalk.legKeys, InvAByWalk.legValues)
    end
    methods
        function obj = InvAByWalk(transformationDirectory)
            obj@InvABy(transformationDirectory);
            obj.save_invaby(obj.mode, obj.numLegs);
        end
        function save_invaby(obj, mode, numActiveLegs)
            temp = SaveTransformation.load_data(ABy, mode, obj.transformationDirectory);
            ALeg = temp.ALeg;
            ByLeg = temp.ByLeg;
            ABody = temp.ABody;
            ByBody = temp.ByBody;

            invALeg = cell(numActiveLegs,1);
            vectorXLeg = cell(numActiveLegs,1);
            invABody = cell(numActiveLegs,1);
            vectorXBody = cell(numActiveLegs,1);

            bodyInv = 0;
            for activeLeg = 1:numActiveLegs
                % leg
                disp('=== LEG ====')
                disp(activeLeg)
                A = ALeg{activeLeg};
                By = ByLeg{activeLeg};
                A = Utils.sub_values_from_map(A, InvAByWalk.baseConstants);
                A = Utils.sub_values_from_map(A, InvAByWalk.bodyConstants);
                invA = inv(A);
                invA = simplify(invA);
                invALeg(activeLeg) = {invA};
                vectorXLeg(activeLeg) = {simplify(invA*By)};
                % body
                % warning: does not finish if 18x18 matrix
                A = ABody{activeLeg};
                if length(A) < 7 || bodyInv == 1
                    disp('=== BODY ====')
                    A = Utils.sub_values_from_map(A, InvAByWalk.legConstants);
                    disp('after sub')
                    disp(A)
                    invA = inv(A);
                    invA = simplify(invA);
                    invABody(activeLeg) = {invA};
                    vectorXBody(activeLeg) = {simplify(invA*By)};
                end
            end

            fileName = obj.get_filename(obj.mode);
            save(fileName, 'invALeg', 'invABody', 'vectorXLeg', 'vectorXBody', 'ByLeg', 'ByBody');
        end
    end
end