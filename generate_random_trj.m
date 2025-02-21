% Define number of movements
num_movements = 20;
joint_min = -pi;
joint_max = pi;

% Open file to write
fileID = fopen('random_trj.txt', 'w');

% Write initial pause
fprintf(fileID, 'pause: 1\n');

% Generate random movements
for i = 1:num_movements
    joint1 = (joint_max - joint_min) * rand() + joint_min; % Random value in [-pi, pi]
    joint2 = (joint_max - joint_min) * rand() + joint_min;
    
    fprintf(fileID, 'move: [%f, %f]\n', joint1, joint2);
end

% Write final pause
fprintf(fileID, 'pause: 1\n');

% Close file
fclose(fileID);

disp('Movement list saved as "random_trj.txt"');
