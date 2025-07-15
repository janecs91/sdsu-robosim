%addpath('transformations');

%% Initialize saved matrices/functions
% Run this only to update the saved matrices/functions in 'saved' folder (or if none exist)
generate_transformation = true;
generate_aby = false;
generate_functions = false;
transformationDirectory = 'mat2';
functionDirectory = 'functions2';

%% Generate symbolic matrices: t_BE, dt_BB
if generate_transformation == true
    Roll(transformationDirectory).save_dtbb();
    Walk(transformationDirectory).save_dtbb();
    Pull(transformationDirectory).save_dtbb();
end

%% Generate symbolic matrices: A, By 
if generate_aby == true
    AByRoll(transformationDirectory);
    %{
    AByRoll(transformationDirectory);
    AByWalk(transformationDirectory);
    AByPull(transformationDirectory);
    %}
end

%% Generate symbolic inverse matrices? (optional)


%% Convert symbolic matrices into functions (for speed-up)
modes = {'roll', 'walk', 'pull'};
modes = {'roll'};
numLegs = {0, 4, 2};
if generate_functions == true
    fg = FunctionGenerator(transformationDirectory, functionDirectory);
    for i=1:size(modes,2)
        mode = modes{i};
        fg.generate_function_aby(mode);
        fg.generate_function_end_positions(mode);
        if numLegs{i}
            fg.generate_function_rotation_matrix_leg(mode, numLegs{i});
        end
        fg.generate_function_joint_positions(mode);
    end
end


