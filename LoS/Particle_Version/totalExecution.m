%% UAV 위치 측위 방식 비교 분석 스크립트
% TDoA, TWR, Hybrid 세 가지 위치 측위 방식을 비교하는 스크립트
% 각 방식을 10번씩 실행하고 결과를 수집하여 비교 분석합니다.

% 결과 저장 폴더 생성
if ~exist('results', 'dir')
    mkdir('results');
end

% 실험 설정
num_runs = 10;               % 각 방식별 실행 횟수
simulation_time = 30;        % 시뮬레이션 시간 (초)
update_rate = 100;           % 업데이트 속도 (Hz)
num_particles = 500;         % 파티클 필터의 파티클 수
resample_threshold_ratio = 2; % 리샘플링 임계값 비율 (NumParticles/resample_threshold_ratio)
desired_measurements = 100;  % 원하는 측위 횟수
measurement_interval = simulation_time / desired_measurements; % 측위 간격 계산
measurement_interval = 0.05; %측위 간격 0.05초

% 앵커 위치 설정 (모든 시뮬레이션에서 동일)
anchor_positions = [
    0 0 -4.5;    % 앵커 1: 북서쪽 천장
    0 15 -4.5;   % 앵커 2: 북동쪽 천장
    15 0 0;      % 앵커 3: 남서쪽 바닥 근처
    15 15 0;     % 앵커 4: 남동쪽 바닥 근처
    7.5 7.5 -5;  % 앵커 5: 중앙 천장
];

% 기준 앵커 설정 (TDoA 계산에 사용)
reference_anchor_idx = 4;

% 드론 경로 정의 (모든 시뮬레이션에서 동일)
% 첫 번째 태그 드론의 경로
waypoints1 = [5 5 -1.0; 5 10 -2.0; 10 10 -3.0; 10 5 -3.5; 5 5 -4.0; 5 10 -4.5];
time_of_arrival1 = [0 6 12 18 24 30];
initial_position1 = [5 5 -2.5];

% 두 번째 태그 드론의 경로
waypoints2 = [10 5 -2.0; 10 10 -2.5; 5 10 -3.0; 5 5 -3.5; 10 5 -4.0; 10 10 -4.5];
time_of_arrival2 = [0 6 12 18 24 30];
initial_position2 = [10 5 -3.0];

% 결과 저장용 구조체 초기화
results = struct();
results.TDoA = struct('RunErrors', cell(1, num_runs), 'MeanErrors', zeros(1, num_runs), ...
                     'MaxErrors', zeros(1, num_runs), 'XErrors', zeros(1, num_runs), ...
                     'YErrors', zeros(1, num_runs), 'ZErrors', zeros(1, num_runs));
results.TWR = struct('RunErrors', cell(1, num_runs), 'MeanErrors', zeros(1, num_runs), ...
                    'MaxErrors', zeros(1, num_runs), 'XErrors', zeros(1, num_runs), ...
                    'YErrors', zeros(1, num_runs), 'ZErrors', zeros(1, num_runs));
results.Hybrid = struct('RunErrors', cell(1, num_runs), 'MeanErrors', zeros(1, num_runs), ...
                       'MaxErrors', zeros(1, num_runs), 'XErrors', zeros(1, num_runs), ...
                       'YErrors', zeros(1, num_runs), 'ZErrors', zeros(1, num_runs));

% 시뮬레이션 실행 함수 정의
% TDoA 시뮬레이션 실행 함수
function [tag_results] = run_tdoa_simulation(initial_position1, waypoints1, time_of_arrival1, ...
                                            initial_position2, waypoints2, time_of_arrival2, ...
                                            anchor_positions, reference_anchor_idx, ...
                                            sim_time, update_rate, num_particles, resample_threshold_ratio)
    % 시나리오 생성
    Scenario = uavScenario("StopTime", sim_time, "UpdateRate", update_rate, "MaxNumFrames", 20);
    
    % 앵커 생성
    anchors = [];
    anchor_uwb = [];
    anchor_sensor_models = cell(size(anchor_positions, 1), 1);
    
    for i = 1:size(anchor_positions, 1)
        % 앵커 플랫폼 생성
        anchor_i = uavPlatform(['Anchor', num2str(i)], Scenario, ...
            'ReferenceFrame', 'NED', ...
            'InitialPosition', anchor_positions(i,:));
        
        % 앵커에 UWB 수신기 장착
        sensor_model = uavUWB(i, 'rx');
        sensor_model.DetectionThreshold = -100; % 매우 낮게 설정하여 항상 검출되도록
        anchor_sensor_models{i} = sensor_model;
        uwb_i = uavSensor('UWB', anchor_i, sensor_model);
        
        % 배열에 추가
        anchors = [anchors, anchor_i];
        anchor_uwb = [anchor_uwb, uwb_i];
        
        % 앵커 시각화
        updateMesh(anchor_i, 'cuboid', {[0.5 0.5 0.5]}, [0 0 0], [0 0 0], eul2quat([0 0 0]));
    end
    
    % 태그 생성
    tag1 = UAVTag_ParticleTDoA(1, Scenario, initial_position1, waypoints1, time_of_arrival1);
    tag2 = UAVTag_ParticleTDoA(2, Scenario, initial_position2, waypoints2, time_of_arrival2);
    
    % 파티클 필터 초기화 및 리샘플링 임계값 설정
    tag1.initParticleFilter(num_particles);
    tag2.initParticleFilter(num_particles);
    
    % 리샘플링 임계값 설정 (NumParticles/resample_threshold_ratio)
    % UAVTag_ParticleTDoA 클래스의 resampleParticles 함수에서 
    % resampleThreshold = obj.NumParticles / resample_threshold_ratio 로 설정
    
    % TWR 비활성화
    tag1.TWREnabled = false;
    tag2.TWREnabled = false;
    
    % 배열로 관리
    tags = [tag1, tag2];
    
    % 시뮬레이션 세팅
    setup(Scenario);
    
    % 시뮬레이션 루프
    while true
        % 시나리오 진행
        isRunning = advance(Scenario);
        if ~isRunning
            break;
        end
        
        % 현재 시간 가져오기
        t = Scenario.CurrentTime;
        
        % 각 태그 드론 처리
        for i = 1:length(tags)
            tags(i).processStep(t, anchor_sensor_models, anchor_positions, reference_anchor_idx, tags);
        end
    end
    
    % 결과 수집
    tag_results = cell(1, length(tags));
    for i = 1:length(tags)
        stats = tags(i).getStats();
        tag_results{i} = stats;
    end
end

% TWR 시뮬레이션 실행 함수
function [tag_results] = run_twr_simulation(initial_position1, waypoints1, time_of_arrival1, ...
                                           initial_position2, waypoints2, time_of_arrival2, ...
                                           anchor_positions, sim_time, update_rate, ...
                                           num_particles, resample_threshold_ratio)
    % 시나리오 생성
    Scenario = uavScenario("StopTime", sim_time, "UpdateRate", update_rate, "MaxNumFrames", 20);
    
    % 앵커 생성
    anchors = [];
    anchor_uwb = [];
    anchor_sensor_models = cell(size(anchor_positions, 1), 1);
    
    for i = 1:size(anchor_positions, 1)
        % 앵커 플랫폼 생성
        anchor_i = uavPlatform(['Anchor', num2str(i)], Scenario, ...
            'ReferenceFrame', 'NED', ...
            'InitialPosition', anchor_positions(i,:));
        
        % 앵커에 UWB 송수신기 장착 (TWR 모드로 설정)
        sensor_model = uavUWB(i, 'txrx', true);
        sensor_model.ProcessingDelay = 0.001;
        sensor_model.DetectionThreshold = -100;
        anchor_sensor_models{i} = sensor_model;
        uwb_i = uavSensor(['UWB_TWR_Anchor', num2str(i)], anchor_i, sensor_model);
        
        % 배열에 추가
        anchors = [anchors, anchor_i];
        anchor_uwb = [anchor_uwb, uwb_i];
        
        % 앵커 시각화
        updateMesh(anchor_i, 'cuboid', {[0.5 0.5 0.5]}, [0 0 0], [0 0 0], eul2quat([0 0 0]));
    end
    
    % 태그 생성
    tag1 = UAVTag_ParticleTWR(1, Scenario, initial_position1, waypoints1, time_of_arrival1);
    tag2 = UAVTag_ParticleTWR(2, Scenario, initial_position2, waypoints2, time_of_arrival2);
    
    % 파티클 필터 초기화 및 리샘플링 임계값 설정
    tag1.initParticleFilter(num_particles);
    tag2.initParticleFilter(num_particles);
    
    % 리샘플링 임계값 설정 (파티클 필터 클래스 내부에서 설정)
    % UAVTag_ParticleTWR 클래스의 resampleParticles 함수에서 
    % resampleThreshold = obj.NumParticles / resample_threshold_ratio 로 설정
    
    % 배열로 관리
    tags = [tag1, tag2];
    
    % 시뮬레이션 세팅
    setup(Scenario);
    
    % 시뮬레이션 루프
    while true
        % 시나리오 진행
        isRunning = advance(Scenario);
        if ~isRunning
            break;
        end
        
        % 현재 시간 가져오기
        t = Scenario.CurrentTime;
        
        % 각 태그 드론 처리
        for i = 1:length(tags)
            tags(i).processStep(t, anchors, anchor_sensor_models, anchor_positions);
        end
    end
    
    % 결과 수집
    tag_results = cell(1, length(tags));
    for i = 1:length(tags)
        stats = tags(i).getStats();
        tag_results{i} = stats;
    end
end

% Hybrid 시뮬레이션 실행 함수
function [tag_results] = run_hybrid_simulation(initial_position1, waypoints1, time_of_arrival1, ...
                                              initial_position2, waypoints2, time_of_arrival2, ...
                                              anchor_positions, reference_anchor_idx, ...
                                              sim_time, update_rate, num_particles, resample_threshold_ratio)
    % 시나리오 생성
    Scenario = uavScenario("StopTime", sim_time, "UpdateRate", update_rate, "MaxNumFrames", 20);
    
    % 앵커 생성
    anchors = [];
    anchor_uwb = [];
    anchor_sensor_models = cell(size(anchor_positions, 1), 1);
    
    for i = 1:size(anchor_positions, 1)
        % 앵커 플랫폼 생성
        anchor_i = uavPlatform(['Anchor', num2str(i)], Scenario, ...
            'ReferenceFrame', 'NED', ...
            'InitialPosition', anchor_positions(i,:));
        
        % 앵커에 UWB 수신기 장착
        sensor_model = uavUWB(i, 'rx');
        sensor_model.DetectionThreshold = -100;
        anchor_sensor_models{i} = sensor_model;
        uwb_i = uavSensor('UWB', anchor_i, sensor_model);
        
        % 배열에 추가
        anchors = [anchors, anchor_i];
        anchor_uwb = [anchor_uwb, uwb_i];
        
        % 앵커 시각화
        updateMesh(anchor_i, 'cuboid', {[0.5 0.5 0.5]}, [0 0 0], [0 0 0], eul2quat([0 0 0]));
    end
    
    % 태그 생성
    tag1 = UAVTag_Hybrid(1, Scenario, initial_position1, waypoints1, time_of_arrival1);
    tag2 = UAVTag_Hybrid(2, Scenario, initial_position2, waypoints2, time_of_arrival2);
    
    % 파티클 필터 초기화 및 리샘플링 임계값 설정
    tag1.initParticleFilter(num_particles);
    tag2.initParticleFilter(num_particles);
    
    % 리샘플링 임계값 설정
    % UAVTag_Hybrid 클래스의 resampleParticles 함수에서 
    % resampleThreshold = obj.NumParticles / resample_threshold_ratio 로 설정
    
    % 배열로 관리
    tags = [tag1, tag2];
    
    % 시뮬레이션 세팅
    setup(Scenario);
    
    % 시뮬레이션 루프
    while true
        % 시나리오 진행
        isRunning = advance(Scenario);
        if ~isRunning
            break;
        end
        
        % 현재 시간 가져오기
        t = Scenario.CurrentTime;
        
        % 각 태그 드론 처리
        for i = 1:length(tags)
            tags(i).processStep(t, anchor_sensor_models, anchor_positions, reference_anchor_idx, tags);
        end
    end
    
    % 결과 수집
    tag_results = cell(1, length(tags));
    for i = 1:length(tags)
        stats = tags(i).getStats();
        tag_results{i} = stats;
    end
end

%% 메인 시뮬레이션 실행 루프
fprintf('UAV 위치 측위 방식 비교 분석 시작\n');
fprintf('각 방식별 %d번 실행, 총 %d회 시뮬레이션\n', num_runs, 3*num_runs);

% TDoA 시뮬레이션 실행
fprintf('\n=== TDoA 시뮬레이션 시작 ===\n');
for run = 1:num_runs
    fprintf('TDoA 시뮬레이션 %d/%d 실행 중...\n', run, num_runs);
    
    % TDoA 시뮬레이션 실행 및 결과 수집
    tag_results = run_tdoa_simulation(initial_position1, waypoints1, time_of_arrival1, ...
                                     initial_position2, waypoints2, time_of_arrival2, ...
                                     anchor_positions, reference_anchor_idx, ...
                                     simulation_time, update_rate, num_particles, resample_threshold_ratio);
    
    % 결과 저장 (첫 번째 태그 드론만 사용)
    results = struct();
    results.TDoA = struct();
    results.TDoA.RunErrors = cell(1, num_runs);  % 명시적으로 셀 배열로 초기화
    results.TDoA.MeanErrors = zeros(1, num_runs);
    results.TDoA.MaxErrors = zeros(1, num_runs);
    results.TDoA.XErrors = zeros(1, num_runs);
    results.TDoA.YErrors = zeros(1, num_runs);
    results.TDoA.ZErrors = zeros(1, num_runs);
    
    fprintf('TDoA 시뮬레이션 %d/%d 완료. 평균 오차: %.3f m\n', run, num_runs, results.TDoA.MeanErrors(run));
end

% TWR 시뮬레이션 실행
fprintf('\n=== TWR 시뮬레이션 시작 ===\n');
for run = 1:num_runs
    fprintf('TWR 시뮬레이션 %d/%d 실행 중...\n', run, num_runs);
    
    % TWR 시뮬레이션 실행 및 결과 수집
    tag_results = run_twr_simulation(initial_position1, waypoints1, time_of_arrival1, ...
                                    initial_position2, waypoints2, time_of_arrival2, ...
                                    anchor_positions, simulation_time, update_rate, ...
                                    num_particles, resample_threshold_ratio);
    
    % 결과 저장 (첫 번째 태그 드론만 사용)
    results.TWR = struct();
    results.TWR.RunErrors = cell(1, num_runs);  % 명시적으로 셀 배열로 초기화
    results.TWR.MeanErrors = zeros(1, num_runs);
    results.TWR.MaxErrors = zeros(1, num_runs);
    results.TWR.XErrors = zeros(1, num_runs);
    results.TWR.YErrors = zeros(1, num_runs);
    results.TWR.ZErrors = zeros(1, num_runs);
    
    fprintf('TWR 시뮬레이션 %d/%d 완료. 평균 오차: %.3f m\n', run, num_runs, results.TWR.MeanErrors(run));
end

% Hybrid 시뮬레이션 실행
fprintf('\n=== Hybrid 시뮬레이션 시작 ===\n');
for run = 1:num_runs
    fprintf('Hybrid 시뮬레이션 %d/%d 실행 중...\n', run, num_runs);
    
    % Hybrid 시뮬레이션 실행 및 결과 수집
    tag_results = run_hybrid_simulation(initial_position1, waypoints1, time_of_arrival1, ...
                                       initial_position2, waypoints2, time_of_arrival2, ...
                                       anchor_positions, reference_anchor_idx, ...
                                       simulation_time, update_rate, num_particles, resample_threshold_ratio);
    
    % 결과 저장 (첫 번째 태그 드론만 사용)
    results.Hybrid = struct();
    results.Hybrid.RunErrors = cell(1, num_runs);  % 명시적으로 셀 배열로 초기화
    results.Hybrid.MeanErrors = zeros(1, num_runs);
    results.Hybrid.MaxErrors = zeros(1, num_runs);
    results.Hybrid.XErrors = zeros(1, num_runs);
    results.Hybrid.YErrors = zeros(1, num_runs);
    results.Hybrid.ZErrors = zeros(1, num_runs);
    
    fprintf('Hybrid 시뮬레이션 %d/%d 완료. 평균 오차: %.3f m\n', run, num_runs, results.Hybrid.MeanErrors(run));
end

%% 결과 분석 및 시각화
fprintf('\n=== 시뮬레이션 결과 분석 ===\n');

% 평균 오차 통계
tdoa_mean = mean(results.TDoA.MeanErrors);
tdoa_std = std(results.TDoA.MeanErrors);
twr_mean = mean(results.TWR.MeanErrors);
twr_std = std(results.TWR.MeanErrors);
hybrid_mean = mean(results.Hybrid.MeanErrors);
hybrid_std = std(results.Hybrid.MeanErrors);

fprintf('TDoA 평균 오차: %.3f ± %.3f m\n', tdoa_mean, tdoa_std);
fprintf('TWR 평균 오차: %.3f ± %.3f m\n', twr_mean, twr_std);
fprintf('Hybrid 평균 오차: %.3f ± %.3f m\n', hybrid_mean, hybrid_std);

% 축별 오차 통계
fprintf('\n축별 평균 오차:\n');
fprintf('TDoA - X: %.3f m, Y: %.3f m, Z: %.3f m\n', ...
    mean(results.TDoA.XErrors), mean(results.TDoA.YErrors), mean(results.TDoA.ZErrors));
fprintf('TWR - X: %.3f m, Y: %.3f m, Z: %.3f m\n', ...
    mean(results.TWR.XErrors), mean(results.TWR.YErrors), mean(results.TWR.ZErrors));
fprintf('Hybrid - X: %.3f m, Y: %.3f m, Z: %.3f m\n', ...
    mean(results.Hybrid.XErrors), mean(results.Hybrid.YErrors), mean(results.Hybrid.ZErrors));

% 결과 저장
save('results/comparison_results.mat', 'results');

%% 결과 그래프 생성
% 평균 오차 비교 그래프
figure('Name', '위치 측위 방식별 평균 오차 비교');

% 평균 오차 막대 그래프
subplot(2, 2, 1);
bar([tdoa_mean, twr_mean, hybrid_mean]);
hold on;
errorbar(1:3, [tdoa_mean, twr_mean, hybrid_mean], ...
         [tdoa_std, twr_std, hybrid_std], 'k', 'LineStyle', 'none', 'LineWidth', 1.5);
title('평균 위치 추정 오차');
xlabel('측위 방식');
xticklabels({'TDoA', 'TWR', 'Hybrid'});
ylabel('오차 (m)');
grid on;

% 축별 오차 그래프
subplot(2, 2, 2);
bar([mean(results.TDoA.XErrors), mean(results.TWR.XErrors), mean(results.Hybrid.XErrors); ...
     mean(results.TDoA.YErrors), mean(results.TWR.YErrors), mean(results.Hybrid.YErrors); ...
     mean(results.TDoA.ZErrors), mean(results.TWR.ZErrors), mean(results.Hybrid.ZErrors)]');
title('축별 평균 오차');
xlabel('측위 방식');
xticklabels({'TDoA', 'TWR', 'Hybrid'});
ylabel('오차 (m)');
legend('X축', 'Y축', 'Z축');
grid on;

% 오차 분포 박스 플롯
subplot(2, 2, 3);
boxplot([results.TDoA.MeanErrors', results.TWR.MeanErrors', results.Hybrid.MeanErrors'], ...
        'Labels', {'TDoA', 'TWR', 'Hybrid'});
title('오차 분포');
ylabel('오차 (m)');
grid on;

% 최대 오차 비교
subplot(2, 2, 4);
bar([mean(results.TDoA.MaxErrors), mean(results.TWR.MaxErrors), mean(results.Hybrid.MaxErrors)]);
title('평균 최대 오차');
xlabel('측위 방식');
xticklabels({'TDoA', 'TWR', 'Hybrid'});
ylabel('오차 (m)');
grid on;

% 그래프 저장
saveas(gcf, 'results/comparison_results.fig');
saveas(gcf, 'results/comparison_results.png');

% 시간에 따른 오차 변화 (각 방식의 첫 번째 실행 결과 사용)
figure('Name', '시간에 따른 위치 추정 오차 변화');
hold on;

% 각 방식별 오차의 시간 변화
plot(1:length(results.TDoA.RunErrors{1}), results.TDoA.RunErrors{1}, 'r-', 'LineWidth', 1.5);
plot(1:length(results.TWR.RunErrors{1}), results.TWR.RunErrors{1}, 'g-', 'LineWidth', 1.5);
plot(1:length(results.Hybrid.RunErrors{1}), results.Hybrid.RunErrors{1}, 'b-', 'LineWidth', 1.5);

title('시간에 따른 위치 추정 오차 변화');
xlabel('시간 스텝');
ylabel('오차 (m)');
legend('TDoA', 'TWR', 'Hybrid');
grid on;

% 그래프 저장
saveas(gcf, 'results/error_over_time.fig');
saveas(gcf, 'results/error_over_time.png');

fprintf('\n분석 완료. 결과가 results 폴더에 저장되었습니다.\n');