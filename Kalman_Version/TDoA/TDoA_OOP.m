%% TDoA 기반 UWB 위치 추적 시뮬레이션 (객체 지향 버전)
% UAVTag 클래스를 사용하여 여러 드론을 관리하는 시뮬레이션

%% 시나리오 생성
Scenario = uavScenario("StopTime", 30, "UpdateRate", 100, "MaxNumFrames", 20);

%% Simulink 세팅 (필요한 경우)
% open_system("UAVTemplate.slx");

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
    
    % 앵커에 UWB 수신기 장착
    sensorModel = uavUWB(i, 'rx'); % 센서 모델 생성
    sensorModel.DetectionThreshold = -100; % 매우 낮게 설정하여 항상 검출되도록
    anchorSensorModels{i} = sensorModel; % 센서 모델 저장
    uwb_i = uavSensor('UWB', anchor_i, sensorModel);
    
    % 배열에 추가
    anchors = [anchors, anchor_i];
    anchorUWB = [anchorUWB, uwb_i];
    
    % 앵커 시각화 (빨간색 큐브)
    updateMesh(anchor_i, 'cuboid', {[0.5 0.5 0.5]}, [0 0 0], [0 0 0], eul2quat([0 0 0]));
end

% 기준 앵커 설정 (TDoA 계산에 사용)
referenceAnchorIdx = 4;

%% 태그 드론 생성 (UAVTag 클래스 사용)
% 첫 번째 태그 드론의 경로 정의
waypoints1 = [5 5 -1.0; 5 10 -2.0; 10 10 -3.0; 10 5 -3.5; 5 5 -4.0; 5 10 -4.5];
timeOfArrival1 = [0 6 12 18 24 30];
initialPosition1 = [5 5 -2.5];

% 두 번째 태그 드론의 경로 정의
waypoints2 = [10 5 -2.0; 10 10 -2.5; 5 10 -3.0; 5 5 -3.5; 10 5 -4.0; 10 10 -4.5];
timeOfArrival2 = [0 6 12 18 24 30];
initialPosition2 = [10 5 -3.0];

% UAVTag 객체 생성
tag1 = UAVTag(1, Scenario, initialPosition1, waypoints1, timeOfArrival1);
tag2 = UAVTag(2, Scenario, initialPosition2, waypoints2, timeOfArrival2);

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
title(ax1, 'UAV 시나리오 (NED 좌표계)');

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
        tags(i).processStep(t, anchorSensorModels, anchorPositions, referenceAnchorIdx);
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
fprintf('\n===== 시뮬레이션 결과 =====\n');

tagColors = containers.Map('KeyType', 'double', 'ValueType', 'any');
tagColors(1) = 'r'; % 첫 번째 태그는 빨간색
tagColors(2) = 'g'; % 두 번째 태그는 초록색

% 현재 태그의 ID로 색상 가져오기
if tagColors.isKey(tags(i).ID)
    tagColor = tagColors(tags(i).ID);
else
    % 지정되지 않은 ID는 기본값 사용
    tagColor = 'k'; % 기본 검정색
end

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
end

% 모든 태그의 3D 경로를 하나의 그래프에 표시
figure('Name', '모든 태그 3D 경로 비교');
hold on;

% 색상 설정
colors = {'g', 'b', 'r', 'm', 'c'};

% 각 태그 경로 표시
for i = 1:length(tags)
    % 각 태그의 TDoA 데이터가 있는지 확인
    if ~isempty(tags(i).TDoAData.EstimatedTime) && ~isempty(tags(i).TDoAData.TagPosition)
        % 추정 시간에 해당하는 실제 위치 계산
        interpolatedTagPositions = zeros(length(tags(i).TDoAData.EstimatedTime), 3);
        for j = 1:length(tags(i).TDoAData.EstimatedTime)
            est_time = tags(i).TDoAData.EstimatedTime(j);
            % 가장 가까운 시간 인덱스 찾기
            [~, idx] = min(abs(tags(i).TDoAData.Time - est_time));
            if ~isempty(idx) && idx <= size(tags(i).TDoAData.TagPosition, 1)
                interpolatedTagPositions(j, :) = tags(i).TDoAData.TagPosition(idx, :);
            end
        end
        
        % 태그 ID에 따라 색상 지정
        if tags(i).ID == 1
            realColor = 'r'; % 빨간색
            estColor = 'r--'; % 빨간색 점선
            realName = '태그 1 실제 경로 (ID: 1)';
            estName = '태그 1 추정 경로 (ID: 1)';
        elseif tags(i).ID == 2
            realColor = 'g'; % 초록색
            estColor = 'g--'; % 초록색 점선
            realName = '태그 2 실제 경로 (ID: 2)';
            estName = '태그 2 추정 경로 (ID: 2)';
        else
            % 기타 ID에 대한 기본 색상
            realColor = 'b'; % 파란색
            estColor = 'b--'; % 파란색 점선
            realName = sprintf('태그 %d 실제 경로 (ID: %d)', tags(i).ID, tags(i).ID);
            estName = sprintf('태그 %d 추정 경로 (ID: %d)', tags(i).ID, tags(i).ID);
        end
        
        % 실제 경로와 추정 경로 그리기
        plot3(interpolatedTagPositions(:,2), interpolatedTagPositions(:,1), -interpolatedTagPositions(:,3), ...
              realColor, 'LineWidth', 2, 'DisplayName', realName);
        plot3(tags(i).TDoAData.EstimatedPosition(:,2), tags(i).TDoAData.EstimatedPosition(:,1), -tags(i).TDoAData.EstimatedPosition(:,3), ...
              estColor, 'LineWidth', 1.5, 'DisplayName', estName);
    end
end

% 앵커 위치 표시
plot3(anchorPositions(:,2), anchorPositions(:,1), -anchorPositions(:,3), ...
      'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k', 'DisplayName', 'UWB 앵커');

% 그래프 설정
grid on;
xlabel('동쪽 (m)'); ylabel('북쪽 (m)'); zlabel('높이 (m)');
title('TDoA 기반 다중 UAV 위치추적 결과');
legend('show', 'Location', 'best');
axis equal;
xlim([0 15]);
ylim([0 15]);
zlim([0 5]);
view(45, 30);