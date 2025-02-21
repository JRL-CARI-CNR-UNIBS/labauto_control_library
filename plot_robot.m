clear all; close all; clc

model_name="scara0";
program_name="test_trj1_20250221114647";
load(model_name+"\tests\"+program_name+".mat")

% Load the robot model
robot = importrobot(model_name+"/model.urdf");
robot.DataFormat = 'column'; % Ensure correct format

% Create figure
figure;
ax = axes;
view([1 .5 .2])
xlim([-1 1])
ylim([-1 1])
zlim([-1 1])
axis manual
grid on
axis vis3d 

% Set up video writer
video_filename = model_name+"\tests\"+program_name+".avi"; % Name of the output video file
video_fps = 50; % Frames per second
steps=1000/video_fps;
v = VideoWriter(video_filename, 'Motion JPEG AVI'); % Create video writer object
v.FrameRate = video_fps; % Set frame rate
open(v); % Open video file for writing

%%
h1 = [];
h2 = [];

for idx = 1:steps:size(joint_position,1)
    delete(findobj(ax, 'Type', 'Patch'))
    delete(findobj(ax, 'Type', 'Line'))
    
    % Define two configurations
    config1 = joint_position(idx,:)'; % Encoder measure
    config2 = link_position(idx,:)'; % Effective link position

    hold on;
    
    % Show first configuration
    show(robot, config1, 'Parent', ax, 'PreservePlot', true);
    h1 = findobj(ax, 'Type', 'Patch'); % Get patch objects

    % Show second configuration
    show(robot, config2, 'Parent', ax, 'PreservePlot', true);
    h2 = findobj(ax, 'Type', 'Patch'); % Get patch objects again
    h2 = setdiff(h2, h1);

    % Apply transparency (alpha) to both configurations
    for i = 1:length(h1)
        h1(i).FaceAlpha = 1; % Solid for the first configuration
    end
    for i = 1:length(h2)
        h2(i).FaceAlpha = 0.4; % Transparent for the second configuration
    end

    hold off;
    drawnow

    % Capture frame and write to video
    frame = getframe(gcf); % Get current figure frame
    writeVideo(v, frame); % Write frame to video file
end

% Close the video file
close(v);

disp('Video saved successfully!');
