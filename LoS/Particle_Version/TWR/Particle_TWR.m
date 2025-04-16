%% TWR 기반 UWB 위치 추적 시뮬레이션 (파티클 필터 버전)
% UAVTag_ParticleTWR 클래스를 사용하여 여러 드론을 관리하는 시뮬레이션

%% 시나리오 생성
Scenario = uavScenario("StopTime", 30, "UpdateRate", 100, "MaxNumFrames", 20);

%% 앵커 드론 생성 (5개의 고정 위치 앵커)
% NED 좌표계: x는 북쪽, y는 동쪽, z는 아래쪽 방향
anchorPositions = [
    0 0 -4.5;    % 앵커 1: 북서쪽 천장
    0 15 -4.5;   % 앵커 2: 북동쪽 천장
    15 0 0;      % 앵커 3: 남서쪽 바닥 근처
    15 15 0;     % 앵커 4: 남동쪽 바닥 근처
    7.5 7.5 -5;  % 앵커 5: 중앙 천장
];

% 앵커 플랫폼 및 센서 생성
anchors = [];
anchorUWB = [];
anchorSensorModels = cell(5, 1);

for i = 1:5
    % 앵커 플랫폼 생성 (NED 좌표계)
    anchor_i = uavPlatform(['Anchor', num2str(i)], Scenario, ...
        'ReferenceFrame', 'NED', ...
        'InitialPosition', anchorPositions(i,:));
    
    % 앵커에 UWB 송수신기 장착 (TWR 모드로 설정)
    sensorModel = uavUWB(i, 'txrx', true); % txrx 모드와 TWR 활성화
    sensorModel.ProcessingDelay = 0.001; % 응답 처리 지연 설정
    sensorModel.DetectionThreshold = -100; % 매우 낮게 설정하여 항상 검출되도록
    anchorSensorModels{i} = sensorModel; % 센서 모델 저장
    uwb_i = uavSensor(['UWB_TWR_Anchor', num2str(i)], anchor_i, sensorModel);
    
    % 배열에 추가
    anchors = [anchors, anchor_i];
    anchorUWB = [anchorUWB, uwb_i];
    
    % 앵커 시각화 (검정색 큐브)
    updateMesh(anchor_i, 'cuboid', {[0.5 0.5 0.5]}, [0 0 0], [0 0 0], eul2quat([0 0 0]));
end

%% 태그 드론 생성 (UAVTag_ParticleTWR 클래스 사용)
% 첫 번째 태그 드론의 경로 정의
waypoints1 = [5 5 -1.0; 5 10 -2.0; 10 10 -3.0; 10 5 -3.5; 5 5 -4.0; 5 10 -4.5];
timeOfArrival1 = [0 6 12 18 24 30];
initialPosition1 = [5 5 -2.5];

% 두 번째 태그 드론의 경로 정의
waypoints2 = [10 5 -2.0; 10 10 -2.5; 5 10 -3.0; 5 5 -3.5; 10 5 -4.0; 10 10 -4.5];
timeOfArrival2 = [0 6 12 18 24 30];
initialPosition2 = [10 5 -3.0];

% UAVTag_ParticleTWR 객체 생성
tag1 = UAVTag_ParticleTWR(1, Scenario, initialPosition1, waypoints1, timeOfArrival1);
tag2 = UAVTag_ParticleTWR(2, Scenario, initialPosition2, waypoints2, timeOfArrival2);

% 파티클 필터 초기화 (각 태그에 500개의 파티클 사용)
tag1.initParticleFilter(500);
tag2.initParticleFilter(500);

% 배열로 관리
tags = [tag1, tag2];

%% 시각화 설정
% 시나리오 시각화
scenarioFig = figure('Name', 'UAV Scenario');
ax1 = show3D(Scenario);
xlim(ax1, [0 15]);
ylim(ax1, [0 15]);
zlim(ax1, [0 5]);
view(ax1, 45, 30);
grid(ax1, 'on');
title(ax1, '파티클 필터 TWR UAV 시나리오 (NED 좌표계)');

%% 시뮬레이션 세팅
setup(Scenario);

%% 시뮬레이션 루프
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
        tags(i).processStep(t, anchors, anchorSensorModels, anchorPositions);
    end
    
    % 주기적으로 시나리오 시각화 업데이트
    if mod(round(t * 100), 10) == 0 && ishandle(scenarioFig) && ishandle(ax1)
        figure(scenarioFig);
        try
            show3D(Scenario, 'Parent', ax1);
            drawnow limitrate;
        catch
            % 오류 발생 시 새로 그리기
            clf(scenarioFig);
            ax1 = show3D(Scenario);
            xlim(ax1, [0 15]);
            ylim(ax1, [0 15]);
            zlim(ax1, [-5 0]);  % NED 좌표계에서 z축 범위 조정
            view(ax1, 45, 30);
            grid(ax1, 'on');
        end
    end
end

%% 결과 분석
fprintf('\n===== 파티클 필터 TWR 시뮬레이션 결과 =====\n');

% 각 태그별 통계 출력
for i = 1:length(tags)
    stats = tags(i).getStats();
    
    fprintf('\n태그 %d 위치 추정 통계:\n', tags(i).ID);
    fprintf('  평균 오차: %.3f 미터\n', stats.MeanError);
    fprintf('  최대 오차: %.3f 미터\n', stats.MaxError);
    fprintf('  최소 오차: %.3f 미터\n', stats.MinError);
    fprintf('  표준편차: %.3f 미터\n', stats.StdError);
    fprintf('  추정 횟수: %d회\n', stats.Count);
    fprintf('  축별 평균 오차 - X: %.3f m, Y: %.3f m, Z: %.3f m\n', ...
        stats.MeanXError, stats.MeanYError, stats.MeanZError);
end

% 각 태그별 결과 그래프 출력
for i = 1:length(tags)
    tags(i).plotResults(anchorPositions);
    tags(i).plotTimeErrorGraph();
    
    % 현재 파티클 분포 시각화
    tags(i).plotParticles(anchorPositions);
end

% 모든 태그의 결과를 하나의 그래프에 표시
tags(1).plotCombinedResults(tags(2:end), anchorPositions);

%% 파티클 필터 성능 분석 그래프

% 시간에 따른 오차 그래프를 하나의 그림에 표시
figure('Name', '시간에 따른 위치 추정 오차 분석');

for i = 1:length(tags)
    subplot(length(tags), 1, i);
    
    % 파티클 필터 오차 시간 추이
    plot(tags(i).TWRData.EstimatedTime, tags(i).PositionErrors, 'g-', 'LineWidth', 1.5);
    hold on;
    
    % 이동 평균선 추가
    window_size = 5;
    if length(tags(i).PositionErrors) >= window_size
        moving_avg = movmean(tags(i).PositionErrors, window_size);
        plot(tags(i).TWRData.EstimatedTime, moving_avg, 'r-', 'LineWidth', 2, 'DisplayName', '이동 평균');
    end
    
    title(sprintf('태그 %d - 파티클 필터 TWR 위치 추정 오차 추이', tags(i).ID));
    xlabel('시간 (초)');
    ylabel('오차 (m)');
    grid on;
    legend('실시간 오차', '이동 평균', 'Location', 'best');
end

% 시뮬레이션 결과 저장 (추후 분석용)
% save('particle_twr_results.mat', 'tags');