%% UAV Localization Total Execution Script for NLoS Environments
% This script runs all three localization methods (TDoA, TWR, and Hybrid)
% in Non-Line-of-Sight (NLoS) environments with configurable parameters
% and collects comparative results

clear all;
close all;
clc;

%% Configuration
% Add all required paths (modify as needed)
addpath('../NLoS/TDoA');
addpath('../NLoS/TWR');
addpath('../NLoS/Hybrid');

% Simulation parameters
config = struct();
config.numRuns = 10;        % Number of runs for each method
config.stopTime = 30;       % Simulation stop time (seconds)
config.updateRate = 100;    % Scenario update rate (Hz)
config.numFrames = 20;      % Maximum number of frames
config.numParticles = 500;  % Number of particles for particle filters

% Anchor positions (NED coordinates)
config.anchorPositions = [
    0 0 -4.5;    % Anchor 1: North-West ceiling
    0 15 -4.5;   % Anchor 2: North-East ceiling
    15 0 0;      % Anchor 3: South-West floor
    15 15 0;     % Anchor 4: South-East floor
    7.5 7.5 -5;  % Anchor 5: Center ceiling
];

% Reference anchor index for TDoA
config.referenceAnchorIdx = 4;

% Obstacle configuration - based on NLoS simulation scripts
config.obstaclePositions = [
    10.0 7.5 -1.5;   % South-West obstacle
    5.0 7.5 -3.0     % North-East obstacle
];
config.obstacleWidthX = 0.8;    % X direction width
config.obstacleWidthY = 4.0;    % Y direction width 
config.obstacleHeight = 4.0;    % Obstacle height

% Tag drone waypoints and arrival times
% Configured for NLoS environment - modified from NLoS simulation examples
config.waypoints1 = [
    2.5 2.5 -3.5; 
    7.5 2.5 -3.8; 
    12.5 2.5 -3.5; 
    12.5 7.5 -3.8; 
    7.5 7.5 -3.5; 
    2.5 7.5 -3.8; 
    2.5 12.5 -3.5; 
    7.5 12.5 -3.8; 
    12.5 12.5 -3.5
];
config.timeOfArrival1 = [0 3.75 7.5 11.25 15 18.75 22.5 26.25 30];
config.initialPosition1 = [2.5 2.5 -3.5];

% Tag 2 waypoints - different height to test multi-level operation
config.waypoints2 = [
    2.5 2.5 -1.5; 
    7.5 2.5 -1.8; 
    12.5 2.5 -1.5; 
    12.5 7.5 -1.8; 
    7.5 7.5 -1.5; 
    2.5 7.5 -1.8; 
    2.5 12.5 -1.5; 
    7.5 12.5 -1.8; 
    12.5 12.5 -1.5
];
config.timeOfArrival2 = [0 3.75 7.5 11.25 15 18.75 22.5 26.25 30];
config.initialPosition2 = [2.5 2.5 -1.5];

% Results storage
results = struct();
results.TDoA = struct('Runs', {}, 'MeanError', {}, 'MaxError', {}, 'MinError', {}, 'StdError', {});
results.TWR = struct('Runs', {}, 'MeanError', {}, 'MaxError', {}, 'MinError', {}, 'StdError', {});
results.Hybrid = struct('Runs', {}, 'MeanError', {}, 'MaxError', {}, 'MinError', {}, 'StdError', {});

%% Run TDoA Simulations in NLoS Environment
fprintf('\n====== Running Particle Filter TDoA Simulations in NLoS Environment (%d runs) ======\n', config.numRuns);

% All tags across all runs
all_tdoa_tags = cell(config.numRuns, 1);

for run = 1:config.numRuns
    fprintf('\nTDoA NLoS Run %d/%d\n', run, config.numRuns);
    
    % Create scenario
    Scenario = uavScenario("StopTime", config.stopTime, "UpdateRate", config.updateRate, "MaxNumFrames", config.numFrames);
    
    % Create anchors
    anchors = [];
    anchorUWB = [];
    anchorSensorModels = cell(size(config.anchorPositions, 1), 1);
    
    for i = 1:size(config.anchorPositions, 1)
        % Create anchor platform (NED coordinates)
        anchor_i = uavPlatform(['Anchor', num2str(i)], Scenario, ...
            'ReferenceFrame', 'NED', ...
            'InitialPosition', config.anchorPositions(i,:));
        
        % Add UWB receiver to anchor
        sensorModel = uavUWB(i, 'rx');
        sensorModel.DetectionThreshold = -120; % Set very low to ensure detection in NLoS
        anchorSensorModels{i} = sensorModel;
        uwb_i = uavSensor('UWB', anchor_i, sensorModel);
        
        % Add to arrays
        anchors = [anchors, anchor_i];
        anchorUWB = [anchorUWB, uwb_i];
        
        % Visualize anchor 
        updateMesh(anchor_i, 'cuboid', {[0.5 0.5 0.5]}, [0 0 0], [0 0 0], eul2quat([0 0 0]));
    end
    
    % Create obstacles
    obstacles = struct();
    for i = 1:size(config.obstaclePositions, 1)
        % Add obstacle mesh to visualization
        addMesh(Scenario, "polygon", ...
            {[config.obstaclePositions(i,1)-config.obstacleWidthX/2 config.obstaclePositions(i,2)-config.obstacleWidthY/2; ...
              config.obstaclePositions(i,1)+config.obstacleWidthX/2 config.obstaclePositions(i,2)-config.obstacleWidthY/2; ...
              config.obstaclePositions(i,1)+config.obstacleWidthX/2 config.obstaclePositions(i,2)+config.obstacleWidthY/2; ...
              config.obstaclePositions(i,1)-config.obstacleWidthX/2 config.obstaclePositions(i,2)+config.obstacleWidthY/2], ...
            [0 config.obstacleHeight]}, 0.651*ones(1,3));
        
        % Create obstacle information for NLoS calculations
        obstacles(i).position = [config.obstaclePositions(i, 1), config.obstaclePositions(i, 2), config.obstaclePositions(i, 3)];
        obstacles(i).dimensions = [config.obstacleWidthX, config.obstacleWidthY, config.obstacleHeight];
    end
    
    % Create tag drones
    tag1 = UAVTag_TDoA(1, Scenario, config.initialPosition1, config.waypoints1, config.timeOfArrival1);
    tag2 = UAVTag_TDoA(2, Scenario, config.initialPosition2, config.waypoints2, config.timeOfArrival2);
    
    % Initialize particle filters
    tag1.initParticleFilter(config.numParticles);
    tag2.initParticleFilter(config.numParticles);
    
    % Disable TWR for pure TDoA
    tag1.TWREnabled = false;
    tag2.TWREnabled = false;
    
    % Manage in array
    tags = [tag1, tag2];
    
    % Setup simulation
    setup(Scenario);
    
    % Simulation loop
    while true
        % Advance scenario
        isRunning = advance(Scenario);
        if ~isRunning
            break;
        end
        
        % Get current time
        t = Scenario.CurrentTime;
        
        % Process each tag drone with obstacles for NLoS effects
        for i = 1:length(tags)
            tags(i).processStep(t, anchorSensorModels, config.anchorPositions, config.referenceAnchorIdx, tags, obstacles);
        end
    end
    
    % Collect results
    runResults = struct();
    
    for i = 1:length(tags)
        stats = tags(i).getStats();
        tagField = ['Tag', num2str(i)];
        runResults.(tagField) = stats;
        
        fprintf('Tag %d Mean Error: %.3f m\n', i, stats.MeanError);
    end
    
    % Store results
    results.TDoA(run).Runs = runResults;
    results.TDoA(run).MeanError = mean([runResults.Tag1.MeanError, runResults.Tag2.MeanError]);
    results.TDoA(run).MaxError = max([runResults.Tag1.MaxError, runResults.Tag2.MaxError]);
    results.TDoA(run).MinError = min([runResults.Tag1.MinError, runResults.Tag2.MinError]);
    results.TDoA(run).StdError = mean([runResults.Tag1.StdError, runResults.Tag2.StdError]);
    
    % Save current run's tags
    all_tdoa_tags{run} = tags;
end

% Collect data from all tags and save to CSV
all_tags = [];
for i = 1:length(all_tdoa_tags)
    all_tags = [all_tags, all_tdoa_tags{i}];
end
collectAndSaveStepErrors(all_tags, 'tdoa_nlos');

% Calculate average results across runs for TDoA
avgTDoA = struct();
avgTDoA.MeanError = mean([results.TDoA.MeanError]);
avgTDoA.MaxError = mean([results.TDoA.MaxError]);
avgTDoA.MinError = mean([results.TDoA.MinError]);
avgTDoA.StdError = mean([results.TDoA.StdError]);

%% Run TWR Simulations in NLoS Environment
fprintf('\n====== Running Particle Filter TWR Simulations in NLoS Environment (%d runs) ======\n', config.numRuns);

% All tags across all runs
all_twr_tags = cell(config.numRuns, 1);

for run = 1:config.numRuns
    fprintf('\nTWR NLoS Run %d/%d\n', run, config.numRuns);
    
    % Create scenario
    Scenario = uavScenario("StopTime", config.stopTime, "UpdateRate", config.updateRate, "MaxNumFrames", config.numFrames);
    
    % Create anchors
    anchors = [];
    anchorUWB = [];
    anchorSensorModels = cell(size(config.anchorPositions, 1), 1);
    
    for i = 1:size(config.anchorPositions, 1)
        % Create anchor platform (NED coordinates)
        anchor_i = uavPlatform(['Anchor', num2str(i)], Scenario, ...
            'ReferenceFrame', 'NED', ...
            'InitialPosition', config.anchorPositions(i,:));
        
        % Add UWB transceiver to anchor with TWR mode
        sensorModel = uavUWB(i, 'txrx', true); % txrx mode with TWR enabled
        sensorModel.ProcessingDelay = 10e-9; % 10 nanoseconds processing delay
        sensorModel.DetectionThreshold = -120; % Set very low to ensure detection in NLoS
        anchorSensorModels{i} = sensorModel;
        uwb_i = uavSensor(['UWB_TWR_Anchor', num2str(i)], anchor_i, sensorModel);
        
        % Add to arrays
        anchors = [anchors, anchor_i];
        anchorUWB = [anchorUWB, uwb_i];
        
        % Visualize anchor
        updateMesh(anchor_i, 'cuboid', {[0.5 0.5 0.5]}, [0 0 0], [0 0 0], eul2quat([0 0 0]));
    end
    
    % Create obstacles
    obstacles = struct();
    for i = 1:size(config.obstaclePositions, 1)
        % Add obstacle mesh to visualization
        addMesh(Scenario, "polygon", ...
            {[config.obstaclePositions(i,1)-config.obstacleWidthX/2 config.obstaclePositions(i,2)-config.obstacleWidthY/2; ...
              config.obstaclePositions(i,1)+config.obstacleWidthX/2 config.obstaclePositions(i,2)-config.obstacleWidthY/2; ...
              config.obstaclePositions(i,1)+config.obstacleWidthX/2 config.obstaclePositions(i,2)+config.obstacleWidthY/2; ...
              config.obstaclePositions(i,1)-config.obstacleWidthX/2 config.obstaclePositions(i,2)+config.obstacleWidthY/2], ...
            [0 config.obstacleHeight]}, 0.651*ones(1,3));
        
        % Create obstacle information for NLoS calculations
        obstacles(i).position = [config.obstaclePositions(i, 1), config.obstaclePositions(i, 2), config.obstaclePositions(i, 3)];
        obstacles(i).dimensions = [config.obstacleWidthX, config.obstacleWidthY, config.obstacleHeight];
    end
    
    % Create tag drones - using NLoSTWR class for TWR in NLoS environments
    tag1 = UAVTag_NLoSTWR(1, Scenario, config.initialPosition1, config.waypoints1, config.timeOfArrival1, 0.05);
    tag2 = UAVTag_NLoSTWR(2, Scenario, config.initialPosition2, config.waypoints2, config.timeOfArrival2, 0.05);
    
    % Initialize particle filters
    tag1.initParticleFilter(config.numParticles);
    tag2.initParticleFilter(config.numParticles);
    
    % Manage in array
    tags = [tag1, tag2];
    
    % Setup simulation
    setup(Scenario);
    
    % Simulation loop
    while true
        % Advance scenario
        isRunning = advance(Scenario);
        if ~isRunning
            break;
        end
        
        % Get current time
        t = Scenario.CurrentTime;
        
        % Process each tag drone with obstacles for NLoS effects
        for i = 1:length(tags)
            tags(i).processStep(t, anchors, anchorSensorModels, config.anchorPositions, obstacles);
        end
    end
    
    % Collect results
    runResults = struct();
    
    for i = 1:length(tags)
        stats = tags(i).getStats();
        tagField = ['Tag', num2str(i)];
        runResults.(tagField) = stats;
        
        fprintf('Tag %d Mean Error: %.3f m\n', i, stats.MeanError);
    end
    
    % Store results
    results.TWR(run).Runs = runResults;
    results.TWR(run).MeanError = mean([runResults.Tag1.MeanError, runResults.Tag2.MeanError]);
    results.TWR(run).MaxError = max([runResults.Tag1.MaxError, runResults.Tag2.MaxError]);
    results.TWR(run).MinError = min([runResults.Tag1.MinError, runResults.Tag2.MinError]);
    results.TWR(run).StdError = mean([runResults.Tag1.StdError, runResults.Tag2.StdError]);
    
    % Save current run's tags
    all_twr_tags{run} = tags;
end

% Collect data from all tags and save to CSV
all_tags = [];
for i = 1:length(all_twr_tags)
    all_tags = [all_tags, all_twr_tags{i}];
end
collectAndSaveStepErrors(all_tags, 'twr_nlos');

% Calculate average results across runs for TWR
avgTWR = struct();
avgTWR.MeanError = mean([results.TWR.MeanError]);
avgTWR.MaxError = mean([results.TWR.MaxError]);
avgTWR.MinError = mean([results.TWR.MinError]);
avgTWR.StdError = mean([results.TWR.StdError]);

%% Run Hybrid Simulations in NLoS Environment
fprintf('\n====== Running Hybrid TDoA-TWR Simulations in NLoS Environment (%d runs) ======\n', config.numRuns);

% All tags across all runs
all_hybrid_tags = cell(config.numRuns, 1);

for run = 1:config.numRuns
    fprintf('\nHybrid NLoS Run %d/%d\n', run, config.numRuns);
    
    % Create scenario
    Scenario = uavScenario("StopTime", config.stopTime, "UpdateRate", config.updateRate, "MaxNumFrames", config.numFrames);
    
    % Create anchors
    anchors = [];
    anchorUWB = [];
    anchorSensorModels = cell(size(config.anchorPositions, 1), 1);
    
    for i = 1:size(config.anchorPositions, 1)
        % Create anchor platform (NED coordinates)
        anchor_i = uavPlatform(['Anchor', num2str(i)], Scenario, ...
            'ReferenceFrame', 'NED', ...
            'InitialPosition', config.anchorPositions(i,:));
        
        % Add UWB receiver to anchor
        sensorModel = uavUWB(i, 'rx');
        sensorModel.DetectionThreshold = -120; % Set very low to ensure detection in NLoS
        anchorSensorModels{i} = sensorModel;
        uwb_i = uavSensor('UWB', anchor_i, sensorModel);
        
        % Add to arrays
        anchors = [anchors, anchor_i];
        anchorUWB = [anchorUWB, uwb_i];
        
        % Visualize anchor
        updateMesh(anchor_i, 'cuboid', {[0.5 0.5 0.5]}, [0 0 0], [0 0 0], eul2quat([0 0 0]));
    end
    
    % Create obstacles
    obstacles = struct();
    for i = 1:size(config.obstaclePositions, 1)
        % Add obstacle mesh to visualization
        addMesh(Scenario, "polygon", ...
            {[config.obstaclePositions(i,1)-config.obstacleWidthX/2 config.obstaclePositions(i,2)-config.obstacleWidthY/2; ...
              config.obstaclePositions(i,1)+config.obstacleWidthX/2 config.obstaclePositions(i,2)-config.obstacleWidthY/2; ...
              config.obstaclePositions(i,1)+config.obstacleWidthX/2 config.obstaclePositions(i,2)+config.obstacleWidthY/2; ...
              config.obstaclePositions(i,1)-config.obstacleWidthX/2 config.obstaclePositions(i,2)+config.obstacleWidthY/2], ...
            [0 config.obstacleHeight]}, 0.651*ones(1,3));
        
        % Create obstacle information for NLoS calculations
        obstacles(i).position = [config.obstaclePositions(i, 1), config.obstaclePositions(i, 2), config.obstaclePositions(i, 3)];
        obstacles(i).dimensions = [config.obstacleWidthX, config.obstacleWidthY, config.obstacleHeight];
    end
    
    % Create tag drones - using UAVTag_Hybrid class for Hybrid in NLoS environments
    tag1 = UAVTag_Hybrid(1, Scenario, config.initialPosition1, config.waypoints1, config.timeOfArrival1);
    tag2 = UAVTag_Hybrid(2, Scenario, config.initialPosition2, config.waypoints2, config.timeOfArrival2);
    
    % Initialize particle filters
    tag1.initParticleFilter(config.numParticles);
    tag2.initParticleFilter(config.numParticles);
    
    % Manage in array
    tags = [tag1, tag2];
    
    % Setup simulation
    setup(Scenario);
    
    % Simulation loop
    while true
        % Advance scenario
        isRunning = advance(Scenario);
        if ~isRunning
            break;
        end
        
        % Get current time
        t = Scenario.CurrentTime;
        
        % Process each tag drone with obstacles for NLoS effects
        for i = 1:length(tags)
            tags(i).processStep(t, anchorSensorModels, config.anchorPositions, config.referenceAnchorIdx, tags, obstacles);
        end
    end
    
    % Collect results
    runResults = struct();
    
    for i = 1:length(tags)
        stats = tags(i).getStats();
        tagField = ['Tag', num2str(i)];
        runResults.(tagField) = stats;
        
        fprintf('Tag %d Mean Error: %.3f m\n', i, stats.MeanError);
    end
    
    % Store results
    results.Hybrid(run).Runs = runResults;
    results.Hybrid(run).MeanError = mean([runResults.Tag1.MeanError, runResults.Tag2.MeanError]);
    results.Hybrid(run).MaxError = max([runResults.Tag1.MaxError, runResults.Tag2.MaxError]);
    results.Hybrid(run).MinError = min([runResults.Tag1.MinError, runResults.Tag2.MinError]);
    results.Hybrid(run).StdError = mean([runResults.Tag1.StdError, runResults.Tag2.StdError]);
    
    % Save current run's tags
    all_hybrid_tags{run} = tags;
end

% Collect data from all tags and save to CSV
all_tags = [];
for i = 1:length(all_hybrid_tags)
    all_tags = [all_tags, all_hybrid_tags{i}];
end
collectAndSaveStepErrors(all_tags, 'hybrid_nlos');

% Calculate average results across runs for Hybrid
avgHybrid = struct();
avgHybrid.MeanError = mean([results.Hybrid.MeanError]);
avgHybrid.MaxError = mean([results.Hybrid.MaxError]);
avgHybrid.MinError = mean([results.Hybrid.MinError]);
avgHybrid.StdError = mean([results.Hybrid.StdError]);

%% Display Summary Results
fprintf('\n\n===== SUMMARY RESULTS OVER %d RUNS (NLoS Environment) =====\n', config.numRuns);

% TDoA results
fprintf('\nParticle Filter TDoA Results in NLoS:\n');
fprintf('  Average Mean Error: %.3f m\n', avgTDoA.MeanError);
fprintf('  Average Max Error: %.3f m\n', avgTDoA.MaxError);
fprintf('  Average Min Error: %.3f m\n', avgTDoA.MinError);
fprintf('  Average Std Deviation: %.3f m\n', avgTDoA.StdError);

% TWR results
fprintf('\nParticle Filter TWR Results in NLoS:\n');
fprintf('  Average Mean Error: %.3f m\n', avgTWR.MeanError);
fprintf('  Average Max Error: %.3f m\n', avgTWR.MaxError);
fprintf('  Average Min Error: %.3f m\n', avgTWR.MinError);
fprintf('  Average Std Deviation: %.3f m\n', avgTWR.StdError);

% Hybrid results
fprintf('\nHybrid TDoA-TWR Results in NLoS:\n');
fprintf('  Average Mean Error: %.3f m\n', avgHybrid.MeanError);
fprintf('  Average Max Error: %.3f m\n', avgHybrid.MaxError);
fprintf('  Average Min Error: %.3f m\n', avgHybrid.MinError);
fprintf('  Average Std Deviation: %.3f m\n', avgHybrid.StdError);

%% Comparative Analysis: LoS vs NLoS
% Load LoS results if available
try
    los_results = load('localization_comparison_results.mat');
    fprintf('\n===== COMPARISON: LoS vs NLoS =====\n');
    
    fprintf('\nTDoA Performance Degradation in NLoS:\n');
    tdoa_degradation = (avgTDoA.MeanError - los_results.avgTDoA.MeanError) / los_results.avgTDoA.MeanError * 100;
    fprintf('  Mean Error Increase: %.2f%%\n', tdoa_degradation);
    
    fprintf('\nTWR Performance Degradation in NLoS:\n');
    twr_degradation = (avgTWR.MeanError - los_results.avgTWR.MeanError) / los_results.avgTWR.MeanError * 100;
    fprintf('  Mean Error Increase: %.2f%%\n', twr_degradation);
    
    fprintf('\nHybrid Performance Degradation in NLoS:\n');
    hybrid_degradation = (avgHybrid.MeanError - los_results.avgHybrid.MeanError) / los_results.avgHybrid.MeanError * 100;
    fprintf('  Mean Error Increase: %.2f%%\n', hybrid_degradation);
   
catch
    fprintf('\nLoS comparison results not available. Run totalExecution.m to generate LoS data.\n');
end

%% Save Results
save('nlos_localization_comparison_results.mat', 'results', 'avgTDoA', 'avgTWR', 'avgHybrid', 'config');
fprintf('\nResults saved to nlos_localization_comparison_results.mat\n');

%% Generate Comparative Visualization
% Create comparison plot
figure('Name', 'Localization Performance in NLoS Environment', 'Position', [100, 100, 800, 600]);

% Bar chart comparing methods
subplot(2,1,1);
methods = {'TDoA', 'TWR', 'Hybrid'};
errors = [avgTDoA.MeanError, avgTWR.MeanError, avgHybrid.MeanError];
std_devs = [avgTDoA.StdError, avgTWR.StdError, avgHybrid.StdError];

bar_handle = bar(errors);
bar_handle.FaceColor = 'flat';
bar_handle.CData(1,:) = [0.8, 0.2, 0.2]; % Red for TDoA
bar_handle.CData(2,:) = [0.2, 0.6, 0.2]; % Green for TWR
bar_handle.CData(3,:) = [0.2, 0.2, 0.8]; % Blue for Hybrid

hold on;
errorbar(1:3, errors, zeros(size(std_devs)), std_devs, '.k');
set(gca, 'XTick', 1:3, 'XTickLabel', methods);
title('Mean Localization Error in NLoS Environment');
ylabel('Error (meters)');
grid on;

% Create LoS vs NLoS comparison if data available
subplot(2,1,2);
try
    los_results = load('localization_comparison_results.mat');
    
    % Group data for LoS and NLoS
    los_errors = [los_results.avgTDoA.MeanError, los_results.avgTWR.MeanError, los_results.avgHybrid.MeanError];
    nlos_errors = [avgTDoA.MeanError, avgTWR.MeanError, avgHybrid.MeanError];
    
    % Create grouped bar chart
    bar_data = [los_errors; nlos_errors]';
    bar_handle = bar(bar_data);
    
    % Set colors
    bar_handle(1).FaceColor = [0.4, 0.7, 0.9]; % Light blue for LoS
    bar_handle(2).FaceColor = [0.9, 0.4, 0.3]; % Light red for NLoS
    
    set(gca, 'XTick', 1:3, 'XTickLabel', methods);
    title('Comparison: LoS vs NLoS Performance');
    ylabel('Error (meters)');
    legend('LoS', 'NLoS', 'Location', 'northwest');
    grid on;
catch
    text(0.5, 0.5, 'LoS comparison data not available', 'HorizontalAlignment', 'center');
end

% Save figure
saveas(gcf, 'nlos_localization_comparison.fig');
saveas(gcf, 'nlos_localization_comparison.png');

%% CSV 저장 함수들
function collectAndSaveStepErrors(tags, method_name)
    % 모든 태그에서 스텝별 오차 데이터를 수집하고 CSV로 저장하는 함수
    % 
    % 입력:
    %   tags - 태그 객체 배열 (UAVTag_TDoA, UAVTag_NLoSTWR, 또는 UAVTag_Hybrid)
    %   method_name - 방법 이름 (예: 'tdoa_nlos', 'twr_nlos', 'hybrid_nlos')
    
    % 모든 태그의 스텝 오차 데이터 수집
    all_step_errors = [];
    
    for i = 1:length(tags)
        % 각 태그의 스텝별 오차 데이터 가져오기
        if isa(tags(i), 'UAVTag_TDoA') || isa(tags(i), 'UAVTag_Hybrid')
            % TDoA 또는 Hybrid 방식의 태그
            times = tags(i).TDoAData.EstimatedTime;
            errors = tags(i).PositionErrors;
        elseif isa(tags(i), 'UAVTag_NLoSTWR')
            % TWR 방식의 태그
            times = tags(i).TWRData.EstimatedTime;
            errors = tags(i).PositionErrors;
        else
            warning('Unsupported tag type for tag %d', i);
            continue;
        end
        
        % 누락된 값이 있는지 확인
        if isempty(times) || isempty(errors)
            warning('No estimation data for tag %d', i);
            continue;
        end
        
        % 스텝 번호 부여 (시간 순서대로 1부터 번호 매김)
        [~, idx] = sort(times);
        sorted_times = times(idx);
        sorted_errors = errors(idx);
        
        % 각 스텝별로 [스텝번호, 태그ID, 오차] 추가
        for j = 1:length(sorted_times)
            all_step_errors = [all_step_errors; j, tags(i).ID, sorted_errors(j)];
        end
    end
    
    % 데이터가 비어있는지 확인
    if isempty(all_step_errors)
        warning('No step error data collected for method %s', method_name);
        return;
    end
    
    % 결과 CSV 파일에 저장
    filename = sprintf('%s_result.csv', method_name);
    
    % 원시 데이터 저장 (모든 태그, 모든 스텝)
    saveRawDataToCSV(all_step_errors, sprintf('%s_raw_data.csv', method_name));
    
    % 각 스텝별 통계 계산 및 저장
    step_stats = computeStepStatistics(all_step_errors);
    saveStepStatsToCSV(step_stats, filename);
    
    fprintf('Step error data for %s method saved to %s\n', method_name, filename);
end

function saveRawDataToCSV(raw_data, filename)
    % 원시 데이터를 CSV 파일로 저장
    %
    % 입력:
    %   raw_data - [step_number, tag_id, error] 형식의 행렬
    %   filename - 저장할 파일 이름
    
    fid = fopen(filename, 'w');
    fprintf(fid, 'step_count,tag_id,error\n');
    
    for i = 1:size(raw_data, 1)
        fprintf(fid, '%d,%d,%.6f\n', raw_data(i,1), raw_data(i,2), raw_data(i,3));
    end
    
    fclose(fid);
end

function step_stats = computeStepStatistics(raw_data)
    % 각 스텝별 통계 계산
    %
    % 입력:
    %   raw_data - [step_number, tag_id, error] 형식의 행렬
    %
    % 출력:
    %   step_stats - [step_number, min, q1, median, q3, max, rmse] 형식의 행렬
    
    % 고유한 스텝 번호 찾기
    unique_steps = unique(raw_data(:,1));
    step_stats = zeros(length(unique_steps), 7);
    
    for i = 1:length(unique_steps)
        step = unique_steps(i);
        step_data = raw_data(raw_data(:,1) == step, 3);  % 현재 스텝의 오차 데이터
        
        % 각 스텝별 통계 계산
        step_stats(i, 1) = step;                         % 스텝 번호
        step_stats(i, 2) = min(step_data);               % 최소값
        step_stats(i, 3) = quantile(step_data, 0.25);    % 1사분위수 (Q1)
        step_stats(i, 4) = median(step_data);            % 중앙값
        step_stats(i, 5) = quantile(step_data, 0.75);    % 3사분위수 (Q3)
        step_stats(i, 6) = max(step_data);               % 최대값
        step_stats(i, 7) = sqrt(mean(step_data.^2));     % RMSE
    end
end

function saveStepStatsToCSV(step_stats, filename)
    % 스텝별 통계를 CSV 파일로 저장
    %
    % 입력:
    %   step_stats - [step_number, min, q1, median, q3, max, rmse] 형식의 행렬
    %   filename - 저장할 파일 이름
    
    fid = fopen(filename, 'w');
    fprintf(fid, 'step_count,min_error,q1_error,median_error,q3_error,max_error,rmse\n');
    
    for i = 1:size(step_stats, 1)
        fprintf(fid, '%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n', ...
            step_stats(i,1), step_stats(i,2), step_stats(i,3), ...
            step_stats(i,4), step_stats(i,5), step_stats(i,6), step_stats(i,7));
    end
    
    fclose(fid);
end

function plotNLoSImpact(tag, obstacles, anchor_positions)
    % NLoS 영향 시각화 함수
    %
    % 입력:
    %   tag - 태그 객체
    %   obstacles - 장애물 구조체 배열
    %   anchor_positions - 앵커 위치 행렬
    
    figure('Name', sprintf('NLoS Impact Analysis - Tag %d', tag.ID), 'Position', [100, 100, 1200, 600]);
    
    % 3D 궤적 및 장애물 시각화
    subplot(1, 2, 1);
    
    % 실제 궤적
    if isa(tag, 'UAVTag_TDoA') || isa(tag, 'UAVTag_Hybrid')
        plot3(tag.TDoAData.TagPosition(:,2), tag.TDoAData.TagPosition(:,1), -tag.TDoAData.TagPosition(:,3), ...
              'g-', 'LineWidth', 2, 'DisplayName', '실제 경로');
        hold on;
        plot3(tag.TDoAData.EstimatedPosition(:,2), tag.TDoAData.EstimatedPosition(:,1), -tag.TDoAData.EstimatedPosition(:,3), ...
              'r--', 'LineWidth', 1.5, 'DisplayName', '추정 경로');
    elseif isa(tag, 'UAVTag_NLoSTWR')
        plot3(tag.TWRData.TagPosition(:,2), tag.TWRData.TagPosition(:,1), -tag.TWRData.TagPosition(:,3), ...
              'g-', 'LineWidth', 2, 'DisplayName', '실제 경로');
        hold on;
        plot3(tag.TWRData.EstimatedPosition(:,2), tag.TWRData.EstimatedPosition(:,1), -tag.TWRData.EstimatedPosition(:,3), ...
              'r--', 'LineWidth', 1.5, 'DisplayName', '추정 경로');
    end
    
    % 앵커 위치 표시
    plot3(anchor_positions(:,2), anchor_positions(:,1), -anchor_positions(:,3), ...
          'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k', 'DisplayName', 'UWB 앵커');
    
    % 장애물 시각화
    for i = 1:length(obstacles)
        pos = obstacles(i).position;
        dim = obstacles(i).dimensions;
        
        % 장애물 경계 계산
        minBound = pos - dim/2;
        maxBound = pos + dim/2;
        
        % 장애물 모서리 좌표 계산
        vertices = [
            minBound(1), minBound(2), minBound(3);
            maxBound(1), minBound(2), minBound(3);
            maxBound(1), maxBound(2), minBound(3);
            minBound(1), maxBound(2), minBound(3);
            minBound(1), minBound(2), maxBound(3);
            maxBound(1), minBound(2), maxBound(3);
            maxBound(1), maxBound(2), maxBound(3);
            minBound(1), maxBound(2), maxBound(3);
        ];
        
        % 장애물 면 인덱스
        faces = [
            1 2 3 4;    % 아래쪽 면
            5 6 7 8;    % 위쪽 면
            1 2 6 5;    % 앞쪽 면
            2 3 7 6;    % 오른쪽 면
            3 4 8 7;    % 뒤쪽 면
            4 1 5 8     % 왼쪽 면
        ];
        
        % NED 좌표계 변환 (y, x, -z)
        plotVertices = [vertices(:,2), vertices(:,1), -vertices(:,3)];
        
        % 장애물 그리기
        patch('Vertices', plotVertices, 'Faces', faces, ...
              'FaceColor', [0.7 0.7 0.7], 'FaceAlpha', 0.4, ...
              'EdgeColor', 'k', 'DisplayName', ['장애물 ' num2str(i)]);
    end
    
    % 그래프 설정
    grid on;
    xlabel('동쪽 (m)'); ylabel('북쪽 (m)'); zlabel('높이 (m)');
    title('NLoS 환경에서의 위치 추적');
    legend('show', 'Location', 'best');
    axis equal;
    xlim([0 15]); ylim([0 15]); zlim([0 5]);
    view(45, 30);
    
    % 오차 분석 그래프
    subplot(1, 2, 2);
    
    % 오차 데이터 수집
    if isa(tag, 'UAVTag_TDoA') || isa(tag, 'UAVTag_Hybrid')
        times = tag.TDoAData.EstimatedTime;
        errors = tag.PositionErrors;
    elseif isa(tag, 'UAVTag_NLoSTWR')
        times = tag.TWRData.EstimatedTime;
        errors = tag.PositionErrors;
    end
    
    % 오차 그래프 그리기
    plot(times, errors, 'b-o', 'LineWidth', 1.5, 'MarkerSize', 4);
    
    % 이동 평균 추가
    hold on;
    window_size = 5; % 이동 평균 윈도우 크기
    if length(errors) >= window_size
        moving_avg = movmean(errors, window_size);
        plot(times, moving_avg, 'r-', 'LineWidth', 2, 'DisplayName', '이동 평균');
    end
    
    % NLoS 구간 분석 및 표시
    % 실제 구현에서는 NLoS 상태 정보를 태그 객체에서 가져와야 함
    
    % 그래프 설정
    grid on;
    xlabel('시간 (초)');
    ylabel('위치 추정 오차 (m)');
    title('NLoS 환경에서의 위치 추정 오차');
    if length(errors) >= window_size
        legend('실시간 오차', '이동 평균', 'Location', 'best');
    end
    
    % 시간 구간 표시 (5초 간격)
    xticks(0:5:30);
end

function generateComparisonBoxPlot(results, config)
    % LoS와 NLoS 환경에서의 방법별 성능 비교 박스 플롯 생성
    %
    % 입력:
    %   results - NLoS 환경 결과 구조체
    %   config - 설정 구조체
    
    try
        % LoS 결과 로드
        los_results = load('localization_comparison_results.mat');
        
        % 결과 데이터 수집
        tdoa_errors_nlos = [];
        twr_errors_nlos = [];
        hybrid_errors_nlos = [];
        
        tdoa_errors_los = [];
        twr_errors_los = [];
        hybrid_errors_los = [];
        
        % NLoS 데이터 수집
        for run = 1:config.numRuns
            % Tag 1 결과
            tdoa_errors_nlos = [tdoa_errors_nlos; results.TDoA(run).Runs.Tag1.PositionErrors];
            twr_errors_nlos = [twr_errors_nlos; results.TWR(run).Runs.Tag1.PositionErrors];
            hybrid_errors_nlos = [hybrid_errors_nlos; results.Hybrid(run).Runs.Tag1.PositionErrors];
            
            % Tag 2 결과
            tdoa_errors_nlos = [tdoa_errors_nlos; results.TDoA(run).Runs.Tag2.PositionErrors];
            twr_errors_nlos = [twr_errors_nlos; results.TWR(run).Runs.Tag2.PositionErrors];
            hybrid_errors_nlos = [hybrid_errors_nlos; results.Hybrid(run).Runs.Tag2.PositionErrors];
        end
        
        % LoS 데이터 수집
        for run = 1:length(los_results.results.TDoA)
            % Tag 1 결과
            if isfield(los_results.results.TDoA(run).Runs.Tag1, 'PositionErrors')
                tdoa_errors_los = [tdoa_errors_los; los_results.results.TDoA(run).Runs.Tag1.PositionErrors];
            end
            
            if isfield(los_results.results.TWR(run).Runs.Tag1, 'PositionErrors')
                twr_errors_los = [twr_errors_los; los_results.results.TWR(run).Runs.Tag1.PositionErrors];
            end
            
            if isfield(los_results.results.Hybrid(run).Runs.Tag1, 'PositionErrors')
                hybrid_errors_los = [hybrid_errors_los; los_results.results.Hybrid(run).Runs.Tag1.PositionErrors];
            end
            
            % Tag 2 결과
            if isfield(los_results.results.TDoA(run).Runs.Tag2, 'PositionErrors')
                tdoa_errors_los = [tdoa_errors_los; los_results.results.TDoA(run).Runs.Tag2.PositionErrors];
            end
            
            if isfield(los_results.results.TWR(run).Runs.Tag2, 'PositionErrors')
                twr_errors_los = [twr_errors_los; los_results.results.TWR(run).Runs.Tag2.PositionErrors];
            end
            
            if isfield(los_results.results.Hybrid(run).Runs.Tag2, 'PositionErrors')
                hybrid_errors_los = [hybrid_errors_los; los_results.results.Hybrid(run).Runs.Tag2.PositionErrors];
            end
        end
        
        % 박스 플롯 생성
        figure('Name', 'LoS vs NLoS Performance Comparison', 'Position', [100, 100, 1000, 600]);
        
        % 데이터 구성
        los_data = {tdoa_errors_los, twr_errors_los, hybrid_errors_los};
        nlos_data = {tdoa_errors_nlos, twr_errors_nlos, hybrid_errors_nlos};
        
        % 각 방법별 박스 플롯 위치
        positions = [1 2; 4 5; 7 8];
        labels = {'TDoA', 'TWR', 'Hybrid'};
        colors = {[0.4 0.7 1], [1 0.4 0.4]}; % LoS: 파란색, NLoS: 빨간색
        
        hold on;
        
        % 각 방법별로 LoS와 NLoS 박스 플롯 생성
        for i = 1:3
            h_los = boxplot(los_data{i}, 'Positions', positions(i,1), 'Colors', colors{1}, 'Width', 0.8);
            h_nlos = boxplot(nlos_data{i}, 'Positions', positions(i,2), 'Colors', colors{2}, 'Width', 0.8);
            
            % 데이터가 없는 경우 처리
            if isempty(los_data{i})
                text(positions(i,1), 1, 'No data', 'HorizontalAlignment', 'center');
            end
            if isempty(nlos_data{i})
                text(positions(i,2), 1, 'No data', 'HorizontalAlignment', 'center');
            end
        end
        
        % 그래프 설정
        xlim([0 9]);
        set(gca, 'XTick', [1.5, 4.5, 7.5], 'XTickLabel', labels);
        title('LoS vs NLoS Positioning Error Comparison');
        ylabel('Error (meters)');
        
        % 범례 생성
        legend_handles = [
            plot(NaN, NaN, 'Color', colors{1}, 'LineWidth', 2),
            plot(NaN, NaN, 'Color', colors{2}, 'LineWidth', 2)
        ];
        legend(legend_handles, {'LoS', 'NLoS'}, 'Location', 'northeast');
        
        grid on;
        
        % Y축 제한 설정 (최대 오차의 110%)
        max_error = max([max(tdoa_errors_los), max(twr_errors_los), max(hybrid_errors_los), ...
                         max(tdoa_errors_nlos), max(twr_errors_nlos), max(hybrid_errors_nlos)]);
        ylim([0, max_error*1.1]);
        
        % 각 방법 간 구분선 추가
        line([3, 3], [0, max_error*1.1], 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);
        line([6, 6], [0, max_error*1.1], 'Color', 'k', 'LineStyle', '--', 'LineWidth', 1);
        
        % 통계 정보 출력
        % TDoA
        text(1.5, max_error*1.05, sprintf('Mean (LoS): %.2fm', mean(tdoa_errors_los)), 'HorizontalAlignment', 'center');
        text(1.5, max_error*1.00, sprintf('Mean (NLoS): %.2fm', mean(tdoa_errors_nlos)), 'HorizontalAlignment', 'center');
        text(1.5, max_error*0.95, sprintf('Degradation: %.1f%%', (mean(tdoa_errors_nlos)/mean(tdoa_errors_los)-1)*100), 'HorizontalAlignment', 'center');
        
        % TWR
        text(4.5, max_error*1.05, sprintf('Mean (LoS): %.2fm', mean(twr_errors_los)), 'HorizontalAlignment', 'center');
        text(4.5, max_error*1.00, sprintf('Mean (NLoS): %.2fm', mean(twr_errors_nlos)), 'HorizontalAlignment', 'center');
        text(4.5, max_error*0.95, sprintf('Degradation: %.1f%%', (mean(twr_errors_nlos)/mean(twr_errors_los)-1)*100), 'HorizontalAlignment', 'center');
        
        % Hybrid
        text(7.5, max_error*1.05, sprintf('Mean (LoS): %.2fm', mean(hybrid_errors_los)), 'HorizontalAlignment', 'center');
        text(7.5, max_error*1.00, sprintf('Mean (NLoS): %.2fm', mean(hybrid_errors_nlos)), 'HorizontalAlignment', 'center');
        text(7.5, max_error*0.95, sprintf('Degradation: %.1f%%', (mean(hybrid_errors_nlos)/mean(hybrid_errors_los)-1)*100), 'HorizontalAlignment', 'center');
        
        % 저장
        saveas(gcf, 'los_vs_nlos_comparison.fig');
        saveas(gcf, 'los_vs_nlos_comparison.png');
        
    catch e
        warning('LoS comparison could not be generated: %s', e.message);
        fprintf('Make sure localization_comparison_results.mat exists by running totalExecution.m first.\n');
    end
end