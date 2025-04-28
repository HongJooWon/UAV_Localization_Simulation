%% UAV Localization Total Execution Script (수정본)
% This script runs all three localization methods (TDoA, TWR, and Hybrid)
% with configurable parameters and collects comparative results

clear all;
close all;
clc;

%% Configuration
% Add all required paths (modify as needed)
addpath('Avg_TDoA');
addpath('Avg_TWR');
addpath('Avg_Hybrid');

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

% Tag drone waypoints and arrival times
% Tag 1 waypoints
config.waypoints1 = [
    5 5 -1.0;     % Start
    5 10 -2.0;    % Point 2
    10 10 -3.0;   % Point 3
    10 5 -3.5;    % Point 4
    5 5 -4.0;     % Point 5
    5 10 -4.5     % End
];
config.timeOfArrival1 = [0 6 12 18 24 30];
config.initialPosition1 = [5 5 -2.5];

% Tag 2 waypoints
config.waypoints2 = [
    10 5 -2.0;    % Start
    10 10 -2.5;   % Point 2
    5 10 -3.0;    % Point 3
    5 5 -3.5;     % Point 4
    10 5 -4.0;    % Point 5
    10 10 -4.5    % End
];
config.timeOfArrival2 = [0 6 12 18 24 30];
config.initialPosition2 = [10 5 -3.0];

% Results storage
results = struct();
results.TDoA = struct('Runs', {}, 'MeanError', {}, 'MaxError', {}, 'MinError', {}, 'StdError', {});
results.TWR = struct('Runs', {}, 'MeanError', {}, 'MaxError', {}, 'MinError', {}, 'StdError', {});
results.Hybrid = struct('Runs', {}, 'MeanError', {}, 'MaxError', {}, 'MinError', {}, 'StdError', {});

%% Run TDoA Simulations
fprintf('\n====== Running Particle Filter TDoA Simulations (%d runs) ======\n', config.numRuns);

% 모든 실행의 태그 저장용 배열
all_tdoa_tags = cell(config.numRuns, 1);

for run = 1:config.numRuns
    fprintf('\nTDoA Run %d/%d\n', run, config.numRuns);
    
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
        sensorModel.DetectionThreshold = -100; % Set very low to ensure detection
        anchorSensorModels{i} = sensorModel;
        uwb_i = uavSensor('UWB', anchor_i, sensorModel);
        
        % Add to arrays
        anchors = [anchors, anchor_i];
        anchorUWB = [anchorUWB, uwb_i];
        
        % Visualize anchor (red cube)
        updateMesh(anchor_i, 'cuboid', {[0.5 0.5 0.5]}, [0 0 0], [0 0 0], eul2quat([0 0 0]));
    end
    
    % Create tag drones
    tag1 = UAVTag_ParticleTDoA(1, Scenario, config.initialPosition1, config.waypoints1, config.timeOfArrival1);
    tag2 = UAVTag_ParticleTDoA(2, Scenario, config.initialPosition2, config.waypoints2, config.timeOfArrival2);
    
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
        
        % Process each tag drone
        for i = 1:length(tags)
            tags(i).processStep(t, anchorSensorModels, config.anchorPositions, config.referenceAnchorIdx, tags);
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
    
    % 현재 실행의 태그들 저장
    all_tdoa_tags{run} = tags;
end

% 모든 태그에서 데이터 수집 및 CSV 저장
all_tags = [];
for i = 1:length(all_tdoa_tags)
    all_tags = [all_tags, all_tdoa_tags{i}];
end
collectAndSaveStepErrors(all_tags, 'tdoa');

% Calculate average results across runs for TDoA
avgTDoA = struct();
avgTDoA.MeanError = mean([results.TDoA.MeanError]);
avgTDoA.MaxError = mean([results.TDoA.MaxError]);
avgTDoA.MinError = mean([results.TDoA.MinError]);
avgTDoA.StdError = mean([results.TDoA.StdError]);

%% Run TWR Simulations
fprintf('\n====== Running Particle Filter TWR Simulations (%d runs) ======\n', config.numRuns);

% 모든 실행의 태그 저장용 배열
all_twr_tags = cell(config.numRuns, 1);

for run = 1:config.numRuns
    fprintf('\nTWR Run %d/%d\n', run, config.numRuns);
    
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
        sensorModel = uavUWB(i, 'txrx', true);
        sensorModel.ProcessingDelay = 0.001; % Set response processing delay
        sensorModel.DetectionThreshold = -100; % Set very low to ensure detection
        anchorSensorModels{i} = sensorModel;
        uwb_i = uavSensor(['UWB_TWR_Anchor', num2str(i)], anchor_i, sensorModel);
        
        % Add to arrays
        anchors = [anchors, anchor_i];
        anchorUWB = [anchorUWB, uwb_i];
        
        % Visualize anchor (black cube)
        updateMesh(anchor_i, 'cuboid', {[0.5 0.5 0.5]}, [0 0 0], [0 0 0], eul2quat([0 0 0]));
    end
    
    % Create tag drones
    tag1 = UAVTag_ParticleTWR(1, Scenario, config.initialPosition1, config.waypoints1, config.timeOfArrival1);
    tag2 = UAVTag_ParticleTWR(2, Scenario, config.initialPosition2, config.waypoints2, config.timeOfArrival2);
    
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
        
        % Process each tag drone
        for i = 1:length(tags)
            tags(i).processStep(t, anchors, anchorSensorModels, config.anchorPositions);
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
    
    % 현재 실행의 태그들 저장
    all_twr_tags{run} = tags;
end

% 모든 태그에서 데이터 수집 및 CSV 저장
all_tags = [];
for i = 1:length(all_twr_tags)
    all_tags = [all_tags, all_twr_tags{i}];
end
collectAndSaveStepErrors(all_tags, 'twr');

% Calculate average results across runs for TWR
avgTWR = struct();
avgTWR.MeanError = mean([results.TWR.MeanError]);
avgTWR.MaxError = mean([results.TWR.MaxError]);
avgTWR.MinError = mean([results.TWR.MinError]);
avgTWR.StdError = mean([results.TWR.StdError]);

%% Run Hybrid Simulations
fprintf('\n====== Running Hybrid TDoA-TWR Simulations (%d runs) ======\n', config.numRuns);

% 모든 실행의 태그 저장용 배열
all_hybrid_tags = cell(config.numRuns, 1);

for run = 1:config.numRuns
    fprintf('\nHybrid Run %d/%d\n', run, config.numRuns);
    
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
        sensorModel.DetectionThreshold = -100; % Set very low to ensure detection
        anchorSensorModels{i} = sensorModel;
        uwb_i = uavSensor('UWB', anchor_i, sensorModel);
        
        % Add to arrays
        anchors = [anchors, anchor_i];
        anchorUWB = [anchorUWB, uwb_i];
        
        % Visualize anchor (red cube)
        updateMesh(anchor_i, 'cuboid', {[0.5 0.5 0.5]}, [0 0 0], [0 0 0], eul2quat([0 0 0]));
    end
    
    % Create tag drones
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
        
        % Process each tag drone
        for i = 1:length(tags)
            tags(i).processStep(t, anchorSensorModels, config.anchorPositions, config.referenceAnchorIdx, tags);
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
    
    % 현재 실행의 태그들 저장
    all_hybrid_tags{run} = tags;
end

% 모든 태그에서 데이터 수집 및 CSV 저장
all_tags = [];
for i = 1:length(all_hybrid_tags)
    all_tags = [all_tags, all_hybrid_tags{i}];
end
collectAndSaveStepErrors(all_tags, 'hybrid');

% Calculate average results across runs for Hybrid
avgHybrid = struct();
avgHybrid.MeanError = mean([results.Hybrid.MeanError]);
avgHybrid.MaxError = mean([results.Hybrid.MaxError]);
avgHybrid.MinError = mean([results.Hybrid.MinError]);
avgHybrid.StdError = mean([results.Hybrid.StdError]);

%% Display Summary Results
fprintf('\n\n===== SUMMARY RESULTS OVER %d RUNS =====\n', config.numRuns);

% TDoA results
fprintf('\nParticle Filter TDoA Results:\n');
fprintf('  Average Mean Error: %.3f m\n', avgTDoA.MeanError);
fprintf('  Average Max Error: %.3f m\n', avgTDoA.MaxError);
fprintf('  Average Min Error: %.3f m\n', avgTDoA.MinError);
fprintf('  Average Std Deviation: %.3f m\n', avgTDoA.StdError);

% TWR results
fprintf('\nParticle Filter TWR Results:\n');
fprintf('  Average Mean Error: %.3f m\n', avgTWR.MeanError);
fprintf('  Average Max Error: %.3f m\n', avgTWR.MaxError);
fprintf('  Average Min Error: %.3f m\n', avgTWR.MinError);
fprintf('  Average Std Deviation: %.3f m\n', avgTWR.StdError);

% Hybrid results
fprintf('\nHybrid TDoA-TWR Results:\n');
fprintf('  Average Mean Error: %.3f m\n', avgHybrid.MeanError);
fprintf('  Average Max Error: %.3f m\n', avgHybrid.MaxError);
fprintf('  Average Min Error: %.3f m\n', avgHybrid.MinError);
fprintf('  Average Std Deviation: %.3f m\n', avgHybrid.StdError);

%% Save Results
save('localization_comparison_results.mat', 'results', 'avgTDoA', 'avgTWR', 'avgHybrid', 'config');
fprintf('\nResults saved to localization_comparison_results.mat\n');

%% CSV 저장 함수들
function collectAndSaveStepErrors(tags, method_name)
    % 모든 태그에서 스텝별 오차 데이터를 수집하고 CSV로 저장하는 함수
    % 
    % 입력:
    %   tags - 태그 객체 배열 (UAVTag_ParticleTDoA, UAVTag_ParticleTWR, 또는 UAVTag_Hybrid)
    %   method_name - 방법 이름 (예: 'tdoa', 'twr', 'hybrid')
    
    % 모든 태그의 스텝 오차 데이터 수집
    all_step_errors = [];
    
    for i = 1:length(tags)
        % 각 태그의 스텝별 오차 데이터 가져오기
        if isa(tags(i), 'UAVTag_ParticleTDoA') || isa(tags(i), 'UAVTag_Hybrid')
            % TDoA 또는 Hybrid 방식의 태그
            times = tags(i).TDoAData.EstimatedTime;
            errors = tags(i).PositionErrors;
        elseif isa(tags(i), 'UAVTag_ParticleTWR')
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
    filename = sprintf('%s_los_result.csv', method_name);
    
    % 원시 데이터 저장 (모든 태그, 모든 스텝)
    saveRawDataToCSV(all_step_errors, sprintf('%s_los_raw_data.csv', method_name));
    
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