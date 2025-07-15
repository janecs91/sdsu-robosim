filepath = 'mat2/tbe_roll.mat';
data = load(filepath);

accumMatrices = data.accumulated_matrices;
accumMatrix1 = accumMatrices{1};
nextMatrices = data.next_matrices;
disp(size(nextMatrices));
nextMatrix1 = nextMatrices{1};
disp(size(nextMatrix1));

i = 5;
disp('accum matrix');
disp(accumMatrix1{i});
disp('=============');
disp('next matrix');
disp(nextMatrix1{i+1});
