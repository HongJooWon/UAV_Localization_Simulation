%% TWR 기반 UWB 위치 추적 시뮬레이션 (NLoS 환경 지원)
% UAVTag_ParticleTWR 클래스를 사용하여 여러 드론을 관리하는 시뮬레이션
% NLoS(Non-Line-of-Sight) 환경에서의 TWR 측위 성능 분석

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
    
    % 앵커에 UWB 송수신기 장착 (TWR 모드)
    sensorModel = uavUWB(i, 'txrx', true); % txrx 모드와 TWR 활성화
    sensorModel.DetectionThreshold = -120; % 매우 낮게 설정하여 항상 검출되도록
    sensorModel.ProcessingDelay = 10e-9; % 10 나노초 처리 지연
    anchorSensorModels{i} = sensorModel; % 센서 모델 저장
    uwb_i = uavSensor(['UWB_Anchor', num2str(i)], anchor_i, sensorModel);
    
    % 배열에 추가
    anchors = [anchors, anchor_i];
    anchorUWB = [anchorUWB, uwb_i];
    
    % 앵커 시각화 (빨간색 큐브)
    updateMesh(anchor_i, 'cuboid', {[0.5 0.5 0.5]}, [0 0 0], [0 0 0], eul2quat([0 0 0]));
end

%% 장애물 정의 및 추가
ObstaclePositions = [
    10.0 7.5 -1.5;   % 남서쪽 장애물
    5.0 7.5 -3.0     % 북동쪽 장애물
];
ObstaclesWidthX = 0.8;   % X 방향 너비 (가로 길이만 늘림)
ObstaclesWidthY = 4.0;   % Y 방향 너비 (기존 유지)
ObstacleHeight = 4.0;    % 장애물 높이

% 시나리오에 장애물 추가
for i = 1:size(ObstaclePositions,1)
    addMesh(Scenario,"polygon", ...
    {[ObstaclePositions(i,1)-ObstaclesWidthX/2 ObstaclePositions(i,2)-ObstaclesWidthY/2; ...
      ObstaclePositions(i,1)+ObstaclesWidthX/2 ObstaclePositions(i,2)-ObstaclesWidthY/2; ...
      ObstaclePositions(i,1)+ObstaclesWidthX/2 ObstaclePositions(i,2)+ObstaclesWidthY/2; ...
      ObstaclePositions(i,1)-ObstaclesWidthX/2 ObstaclePositions(i,2)+ObstaclesWidthY/2], ...
    [0 ObstacleHeight]}, 0.651*ones(1,3));
end

% 장애물 구조체 생성 (NLoS 계산용)
obstacles = struct();
for i = 1:size(ObstaclePositions, 1)
    obstacles(i).position = [ObstaclePositions(i, 1), ObstaclePositions(i, 2), ObstaclePositions(i, 3)];
    obstacles(i).dimensions = [ObstaclesWidthX, ObstaclesWidthY, ObstacleHeight];
end

%% 태그 드론 생성 (UAVTag_ParticleTWR 클래스 사용)
% 첫 번째 태그 드론의 경로 정의
% 첫 번째 태그 드론의 경로 정의
waypoints1 = [2.5 2.5 -3.5; 7.5 2.5 -3.8; 12.5 2.5 -3.5; 12.5 7.5 -3.8; 7.5 7.5 -3.5; 2.5 7.5 -3.8; ...
              2.5 12.5 -3.5; 7.5 12.5 -3.8; 12.5 12.5 -3.5];
timeOfArrival1 = [0 3.75 7.5 11.25 15 18.75 22.5 26.25 30];
initialPosition1 = [2.5 2.5 -3.5];

% 두 번째 태그 드론의 경로 정의
waypoints2 = [2.5 2.5 -1.5; 7.5 2.5 -1.8; 12.5 2.5 -1.5; 12.5 7.5 -1.8; 7.5 7.5 -1.5; 2.5 7.5 -1.8; ...
              2.5 12.5 -1.5; 7.5 12.5 -1.8; 12.5 12.5 -1.5];
timeOfArrival2 = [0 3.75 7.5 11.25 15 18.75 22.5 26.25 30];
initialPosition2 = [2.5 2.5 -1.5];

% 측정 간격 설정 (전역 변수로 정의)
measurement_interval = 0.05; % 50ms마다 측정

% UAVTag_ParticleTWR 객체 생성
tag1 = UAVTag_NLoSTWR(1, Scenario, initialPosition1, waypoints1, timeOfArrival1, measurement_interval);
tag2 = UAVTag_NLoSTWR(2, Scenario, initialPosition2, waypoints2, timeOfArrival2, measurement_interval);

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
title(ax1, 'TWR NLoS 시나리오 (NED 좌표계)');

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
    
    % 각 태그 드론 처리 - obstacles 매개변수 전달
    for i = 1:length(tags)
        tags(i).processStep(t, anchors, anchorSensorModels, anchorPositions, obstacles);
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
fprintf('\n===== TWR NLoS 시뮬레이션 결과 =====\n');

% 기본 통계 출력
for i = 1:length(tags)
    stats = tags(i).getStats();
    
    fprintf('\n태그 %d 위치 추정 통계 (TWR NLoS):\n', tags(i).ID);
    fprintf('  평균 오차: %.3f 미터\n', stats.MeanError);
    fprintf('  최대 오차: %.3f 미터\n', stats.MaxError);
    fprintf('  최소 오차: %.3f 미터\n', stats.MinError);
    fprintf('  표준편차: %.3f 미터\n', stats.StdError);
    fprintf('  추정 횟수: %d회\n', stats.Count);
    fprintf('  축별 평균 오차 - X: %.3f m, Y: %.3f m, Z: %.3f m\n', ...
        stats.MeanXError, stats.MeanYError, stats.MeanZError);
end

% 시간에 따른 위치 추정 오차 그래프 (각 태그별로)
figure('Name', 'TWR NLoS 위치 추정 오차');
for i = 1:length(tags)
    subplot(length(tags), 1, i);
    
    % 오차 데이터가 비어있지 않은지 확인
    if ~isempty(tags(i).PositionErrors) && any(~isnan(tags(i).PositionErrors))
        % 오차 그래프 
        plot(tags(i).TWRData.EstimatedTime, tags(i).PositionErrors, 'Color', [0, 0.5, 0], 'LineWidth', 1.5);

        % y축 범위 설정
        max_error = max(tags(i).PositionErrors(~isnan(tags(i).PositionErrors))) * 1.1;
        if ~isempty(max_error) && ~isnan(max_error) && max_error > 0
            ylim([0, max_error]);
        end
    else
        text(0.5, 0.5, '데이터가 충분하지 않습니다', 'HorizontalAlignment', 'center');
    end
    
    title(sprintf('태그 %d - TWR NLoS 위치 추정 오차', tags(i).ID));
    xlabel('시간 (초)');
    ylabel('오차 (m)');
    grid on;
end

% 3D 궤적 비교 그래프
figure('Name', 'TWR NLoS 위치 추적 결과');
hold on;

% 장애물 표시
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
          'FaceColor', [0.7 0.7 0.7], 'FaceAlpha', 0.5, ...
          'EdgeColor', 'k', 'DisplayName', ['장애물 ' num2str(i)]);
end

% 앵커 위치 표시
plot3(anchorPositions(:,2), anchorPositions(:,1), -anchorPositions(:,3), ...
      'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k', 'DisplayName', 'UWB 앵커');

% 각 태그별 실제 궤적과 추정 궤적 표시
colors = {'r', 'g', 'b', 'm', 'c'};
markers = {'o', 's', 'd', '^', 'v'};

for i = 1:length(tags)
    if ~isempty(tags(i).TWRData.EstimatedTime) && ~isempty(tags(i).TWRData.EstimatedPosition)
        % 추정 시간에 해당하는 실제 위치 계산
        interpolatedTagPositions = zeros(length(tags(i).TWRData.EstimatedTime), 3);
        for j = 1:length(tags(i).TWRData.EstimatedTime)
            est_time = tags(i).TWRData.EstimatedTime(j);
            [~, idx] = min(abs(tags(i).TWRData.Time - est_time));
            if ~isempty(idx) && idx <= size(tags(i).TWRData.TagPosition, 1)
                interpolatedTagPositions(j, :) = tags(i).TWRData.TagPosition(idx, :);
            end
        end
        
        % 실제 궤적
        plot3(interpolatedTagPositions(:,2), interpolatedTagPositions(:,1), -interpolatedTagPositions(:,3), ...
              [colors{i}, '-'], 'LineWidth', 2, 'DisplayName', sprintf('태그 %d 실제 경로', tags(i).ID));
        
        % TWR 추정 궤적
        plot3(tags(i).TWRData.EstimatedPosition(:,2), tags(i).TWRData.EstimatedPosition(:,1), -tags(i).TWRData.EstimatedPosition(:,3), ...
              [colors{i}, '--'], 'LineWidth', 1.5, 'DisplayName', sprintf('태그 %d TWR NLoS 추정 경로', tags(i).ID));
        
        % 시작점과 끝점 표시
        plot3(interpolatedTagPositions(1,2), interpolatedTagPositions(1,1), -interpolatedTagPositions(1,3), ...
              [colors{i}, markers{i}], 'MarkerSize', 10, 'MarkerFaceColor', colors{i}, 'DisplayName', sprintf('태그 %d 시작점', tags(i).ID));
        plot3(interpolatedTagPositions(end,2), interpolatedTagPositions(end,1), -interpolatedTagPositions(end,3), ...
              [colors{i}, markers{i}], 'MarkerSize', 10, 'DisplayName', sprintf('태그 %d 종료점', tags(i).ID));
    end
end

% 그래프 설정
grid on;
xlabel('동쪽 (m)'); 
ylabel('북쪽 (m)'); 
zlabel('높이 (m)');
title('TWR NLoS 기반 다중 UAV 위치추적 결과');
legend('show', 'Location', 'best');
axis equal;
xlim([0 15]);
ylim([0 15]);
zlim([0 5]);
view(45, 30);

% 파티클 분포 시각화
figure('Name', '파티클 분포 시각화');
subplot(1,2,1);
tags(1).plotParticles(anchorPositions, obstacles);
title('태그 1 - 파티클 분포');

subplot(1,2,2);
tags(2).plotParticles(anchorPositions, obstacles);
title('태그 2 - 파티클 분포');