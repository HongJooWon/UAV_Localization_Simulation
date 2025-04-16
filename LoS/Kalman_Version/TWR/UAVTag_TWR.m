classdef UAVTag_TWR < handle
    % UAVTag_TWR UWB TWR 태그가 장착된 UAV 드론 클래스
    % 
    % 이 클래스는 Two-Way Ranging(TWR) 위치 추정 시스템에서 태그 역할을 하는 드론을 객체화합니다.
    % 드론의 움직임, UWB 신호 전송 및 수신, 위치 추정 결과 등을 관리합니다.
    
    properties
        ID                  % 드론의 고유 ID
        Platform            % UAV 플랫폼 객체
        UWBSensor           % UWB 센서 객체
        SensorModel         % UWB 센서 모델
        InitialPosition     % 초기 위치 (NED 좌표계)
        Waypoints           % 경로점
        TimeOfArrival       % 각 경로점 도착 시간
        KalmanState         % 칼만 필터 상태 벡터
        KalmanCovariance    % 칼만 필터 공분산 행렬
        TWRData             % TWR 데이터 저장 구조체
        MeasuredDistances   % 각 앵커와의 측정된 거리
        RequestInterval     % TWR 요청 간격
        LastRequestTime     % 마지막 TWR 요청 시간
        PositionErrors      % 위치 추정 오차 기록
        Color               % 시각화를 위한 색상
    end
    
    methods
        function obj = UAVTag_TWR(id, scenario, initialPosition, waypoints, timeOfArrival)
            % UAVTag_TWR 생성자: 태그 드론 객체를 초기화합니다.
            %
            % 입력:
            %   id              - 드론의 고유 ID
            %   scenario        - UAV 시나리오 객체
            %   initialPosition - 초기 위치 [x, y, z] (NED 좌표계)
            %   waypoints       - 경로점 행렬 (각 행은 [x, y, z] 위치)
            %   timeOfArrival   - 각 경로점에 도착하는 시간 (초)
            
            % 기본 속성 초기화
            obj.ID = id;
            obj.InitialPosition = initialPosition;
            obj.Waypoints = waypoints;
            obj.TimeOfArrival = timeOfArrival;
            
            % 드론 색상 설정 (ID에 따라 다른 색상)
            colorIndex = mod(id-1, 6) + 1;
            colorMap = [1 0 0;     % 빨간색
                        0 1 0;     % 초록색
                        0 0 1;     % 파란색
                        0 0 0;     % 검정색
                        1 0 1;     % 마젠타
                        0 1 1];    % 시안
            obj.Color = colorMap(colorIndex, :);
            
            % 방향 설정 (웨이포인트 수에 맞게)
            numWaypoints = size(waypoints, 1);
            orientations = quaternion(zeros(numWaypoints, 3), 'eulerd', 'ZYX', 'frame');
            
            % 플랫폼 생성 (고유 이름으로)
            obj.Platform = uavPlatform(['UAV', num2str(id)], scenario, ...
                'InitialPosition', initialPosition, ...
                'InitialOrientation', eul2quat([0 0 0]), ...
                'Trajectory', waypointTrajectory(waypoints, ...
                                  'TimeOfArrival', timeOfArrival, ...
                                  'Orientation', orientations));
            
            % UWB 센서 모델 생성 및 설정 - TWR 모드
            obj.SensorModel = uavUWB(id, 'txrx', true); % txrx 모드와 TWR 활성화
            obj.SensorModel.configureUWB();
            
            % TWR 요청 간격 설정 - 각 드론마다 약간 다르게 (충돌 방지)
            obj.RequestInterval = 0.05;
            obj.SensorModel.RequestInterval = obj.RequestInterval;
            obj.LastRequestTime = -0.1;  % 다른 시작 시간
            
            % UWB 센서 장착
            obj.UWBSensor = uavSensor(['UWB_TWR', num2str(id)], obj.Platform, obj.SensorModel);
            
            % 드론 시각화 (ID에 따라 다른 색상)
            updateMesh(obj.Platform, "quadrotor", {0.8}, obj.Color, eul2tform([0 0 pi]));
            
            % TWR 데이터 구조체 초기화
            obj.TWRData = struct();
            obj.TWRData.Time = [];
            obj.TWRData.TagPosition = [];
            obj.TWRData.EstimatedTime = [];
            obj.TWRData.EstimatedPosition = [];
            obj.MeasuredDistances = zeros(5, 1); % 5개 앵커 기준
            
            % 칼만 필터 초기화
            obj.KalmanState = [initialPosition'; 0; 0; 0]; % 위치와 속도
            obj.KalmanCovariance = diag([10, 10, 10, 1, 1, 1]); % 초기 불확실성
            
            % 오차 기록 초기화
            obj.PositionErrors = [];
            
            % 초기화 완료 메시지
            fprintf('TWR 태그 드론 %d 초기화 완료: 위치 [%.2f, %.2f, %.2f]\n', ...
                id, initialPosition(1), initialPosition(2), initialPosition(3));
        end
        
        function processStep(obj, t, anchorPlatforms, anchorModels, anchorPositions)
            % 시뮬레이션의 한 스텝 처리
            %
            % 입력:
            %   t                - 현재 시뮬레이션 시간 (초)
            %   anchorPlatforms  - 앵커 플랫폼 배열
            %   anchorModels     - 앵커 UWB 센서 모델 셀 배열
            %   anchorPositions  - 앵커 위치 행렬 (각 행은 [x, y, z])
            
            % 현재 드론 위치 가져오기
            [txPos, ~, ~, ~, ~] = lookupPose(obj.Platform.Trajectory, t);
            obj.TWRData.Time(end+1) = t;
            obj.TWRData.TagPosition(end+1, :) = txPos;
            
            % TWR 요청 처리
            % 현재 앵커 인덱스 결정 (라운드 로빈 방식)
            currentAnchorIdx = mod(floor(t / obj.RequestInterval), length(anchorModels)) + 1;
            
            % 새 요청을 보낼 시간인지 확인
            if t - obj.LastRequestTime >= obj.RequestInterval
                % TWR 요청 전송
                [isSent, requestData] = obj.SensorModel.sendTWRRequest(obj.Platform, currentAnchorIdx, t);
                
                if isSent
                    obj.LastRequestTime = t;
                    obj.processTWRRequestResponse(txPos, requestData, currentAnchorIdx, t, anchorPlatforms, anchorModels, anchorPositions);
                end
            end
        end
        
        function processTWRRequestResponse(obj, txPos, requestData, currentAnchorIdx, t, anchorPlatforms, anchorModels, anchorPositions)
            % TWR 요청 및 응답 사이클 처리
            %
            % 입력:
            %   txPos             - 태그의 현재 위치
            %   requestData       - TWR 요청 데이터
            %   currentAnchorIdx  - 현재 앵커 인덱스
            %   t                 - 현재 시간
            %   anchorPlatforms   - 앵커 플랫폼 배열
            %   anchorModels      - 앵커 센서 모델 배열
            %   anchorPositions   - 앵커 위치 배열
            
            try
                % 전파 지연 계산
                distance = norm(txPos - anchorPositions(currentAnchorIdx,:));
                propagationDelay = distance / physconst('LightSpeed');
                
                % 앵커에서의 요청 도착 시간
                requestArrivalTime = t + propagationDelay;
                
                % 앵커의 처리 지연
                anchorProcessingDelay = 0.001; % 가정된 처리 지연, 실제 값은 앵커 모델에서 가져와야 함
                if isfield(anchorModels{currentAnchorIdx}, 'ProcessingDelay')
                    anchorProcessingDelay = anchorModels{currentAnchorIdx}.ProcessingDelay;
                end
                responseTime = requestArrivalTime + anchorProcessingDelay;
                
                % 앵커 응답 생성
                [~, responseData] = anchorModels{currentAnchorIdx}.sendTWRResponse(anchorPlatforms(currentAnchorIdx), requestData, responseTime);
                
                % 태그에서의 응답 도착 시간
                responseArrivalTime = responseTime + propagationDelay;
                
                % 태그에서 응답 처리
                obj.SensorModel.processTWRResponse(responseData, responseArrivalTime);
                
                % 측정된 거리 저장
                fieldName = ['anchor_', num2str(currentAnchorIdx)];
                if isfield(obj.SensorModel.MeasuredDistances, fieldName)
                    obj.MeasuredDistances(currentAnchorIdx) = obj.SensorModel.MeasuredDistances.(fieldName);
                end
                
                % 모든 앵커와의 거리가 측정되었는지 확인
                if all(obj.MeasuredDistances > 0)
                    % 위치 추정
                    estimated_pos = obj.estimatePositionTWR(anchorPositions, obj.MeasuredDistances);
                    
                    if ~any(isnan(estimated_pos))
                        % 칼만 필터 적용
                        obj.applyKalmanFilter(estimated_pos);
                        
                        % 위치 추정 결과 저장
                        obj.TWRData.EstimatedTime(end+1) = t;
                        obj.TWRData.EstimatedPosition(end+1, :) = obj.KalmanState(1:3)';
                        
                        % 오차 계산
                        error = norm(txPos - estimated_pos);
                        obj.PositionErrors(end+1) = error;
                        
                        % 결과 출력
                        fprintf('태그 %d TWR 위치 추정 - 시간: %.2f초\n', obj.ID, t);
                        fprintf('  실제 위치: [%.2f, %.2f, %.2f]\n', txPos(1), txPos(2), txPos(3));
                        fprintf('  추정 위치: [%.2f, %.2f, %.2f]\n', estimated_pos(1), estimated_pos(2), estimated_pos(3));
                        fprintf('  전체 오차: %.2f m\n', error);
                        
                        % 다음 사이클을 위해 거리 재설정
                        obj.MeasuredDistances = zeros(5, 1);
                    end
                end
            catch e
                fprintf('태그 %d TWR 처리 오류: %s\n', obj.ID, e.message);
            end
        end
        
        function applyKalmanFilter(obj, raw_position)
            % 칼만 필터 적용
            %
            % 입력:
            %   raw_position - TWR로 추정된 원시 위치 [x, y, z]
            
            % 상태 전이 행렬 (등속도 모델)
            dt = obj.RequestInterval; % 업데이트 간 시간 단계
            A = [1 0 0 dt 0 0;
                 0 1 0 0 dt 0;
                 0 0 1 0 0 dt;
                 0 0 0 1 0 0;
                 0 0 0 0 1 0;
                 0 0 0 0 0 1];
            
            % 측정 행렬 (위치만 관측)
            H = [1 0 0 0 0 0;
                 0 1 0 0 0 0;
                 0 0 1 0 0 0];
            
            % 프로세스 노이즈 (위치 및 속도 분산)
            Q = diag([0.05, 0.05, 0.05, 0.2, 0.2, 0.2]);
            
            % 측정 노이즈 (위치 측정 분산)
            R = diag([1, 1, 1]);
            
            % 칼만 필터 예측 단계
            obj.KalmanState = A * obj.KalmanState;
            obj.KalmanCovariance = A * obj.KalmanCovariance * A' + Q;
            
            % 측정 업데이트 단계
            twr_raw_position = raw_position';
            
            % 칼만 이득 계산
            K = obj.KalmanCovariance * H' / (H * obj.KalmanCovariance * H' + R);
            
            % 측정값으로 상태 업데이트
            measurement_residual = twr_raw_position - H * obj.KalmanState;
            obj.KalmanState = obj.KalmanState + K * measurement_residual;
            
            % 공분산 업데이트
            obj.KalmanCovariance = (eye(6) - K * H) * obj.KalmanCovariance;
        end
        
        function position = estimatePositionTWR(obj, anchorPositions, distances)
            % TWR 위치 추정
            %
            % 입력:
            %   anchorPositions - 앵커 위치 행렬
            %   distances       - 각 앵커까지의 측정된 거리
            %
            % 출력:
            %   position - 추정된 위치 [x, y, z]
            
            % 초기 추측값 - NED 좌표계에서 앵커들의 중심점
            initial_guess = mean(anchorPositions, 1);
            
            % 비선형 최소제곱법으로 위치 추정
            % NED 좌표계에서 z는 아래 방향으로 음수이므로 상한과 하한을 조정
            lb = [0, 0, -5];    % 하한값 (x, y, z) - 천장
            ub = [15, 15, 0];   % 상한값 (x, y, z) - 바닥
            options = optimoptions('lsqnonlin', 'Display', 'off', 'MaxIterations', 100);
            position = lsqnonlin(@twr_cost_function, initial_guess, lb, ub, options);
            
            % 비용 함수 정의 (내부 함수)
            function residuals = twr_cost_function(pos)
                % 각 앵커와의 이론적 거리 계산
                theoretical_distances = zeros(size(anchorPositions, 1), 1);
                for i = 1:size(anchorPositions, 1)
                    theoretical_distances(i) = norm(pos - anchorPositions(i,:));
                end
                
                % 측정된 거리와 이론적 거리의 차이 반환
                residuals = theoretical_distances - distances;
            end
        end
        
        function plotResults(obj, anchorPositions)
            % 시뮬레이션 결과 그래프 출력
            %
            % 입력:
            %   anchorPositions - 앵커 위치 행렬
            
            % 결과가 없으면 종료
            if isempty(obj.TWRData.EstimatedTime) || isempty(obj.TWRData.EstimatedPosition)
                warning('태그 %d: TWR 위치 추정 데이터가 없습니다. 그래프를 그릴 수 없습니다.', obj.ID);
                return;
            end
            
            % 각 추정 시간에 해당하는 실제 위치 찾기
            interpolatedTagPositions = zeros(length(obj.TWRData.EstimatedTime), 3);
            for i = 1:length(obj.TWRData.EstimatedTime)
                est_time = obj.TWRData.EstimatedTime(i);
                % 가장 가까운 시간 인덱스 찾기
                [~, idx] = min(abs(obj.TWRData.Time - est_time));
                if ~isempty(idx) && idx <= size(obj.TWRData.TagPosition, 1)
                    interpolatedTagPositions(i, :) = obj.TWRData.TagPosition(idx, :);
                end
            end
            
            % 그래프 생성
            figure('Name', sprintf('태그 %d - TWR 위치 추적 결과', obj.ID));
            
            % X, Y, Z 좌표 비교
            subplot(2,2,1);
            plot(obj.TWRData.EstimatedTime, interpolatedTagPositions(:,1), 'g-', ...
                 obj.TWRData.EstimatedTime, obj.TWRData.EstimatedPosition(:,1), 'r--');
            title(sprintf('태그 %d - 북쪽(N) 좌표 비교', obj.ID));
            xlabel('시간 (초)'); ylabel('N 좌표 (m)');
            legend('실제 위치', 'TWR 추정 위치');
            grid on;
            
            subplot(2,2,2);
            plot(obj.TWRData.EstimatedTime, interpolatedTagPositions(:,2), 'g-', ...
                 obj.TWRData.EstimatedTime, obj.TWRData.EstimatedPosition(:,2), 'r--');
            title(sprintf('태그 %d - 동쪽(E) 좌표 비교', obj.ID));
            xlabel('시간 (초)'); ylabel('E 좌표 (m)');
            legend('실제 위치', 'TWR 추정 위치');
            grid on;
            
            subplot(2,2,3);
            plot(obj.TWRData.EstimatedTime, -interpolatedTagPositions(:,3), 'g-', ...
                 obj.TWRData.EstimatedTime, -obj.TWRData.EstimatedPosition(:,3), 'r--');
            title(sprintf('태그 %d - 높이(Height) 좌표 비교', obj.ID));
            xlabel('시간 (초)'); ylabel('높이 (m)');
            legend('실제 위치', 'TWR 추정 위치');
            grid on;
            
            subplot(2,2,4);
            % 위치 오차 계산
            errors = sqrt(sum((interpolatedTagPositions - obj.TWRData.EstimatedPosition).^2, 2));
            plot(obj.TWRData.EstimatedTime, errors, 'b-');
            title(sprintf('태그 %d - TWR 위치 추정 오차', obj.ID));
            xlabel('시간 (초)'); ylabel('오차 (m)');
            grid on;
            
            % 3D 경로 비교 (별도 그림)
            figure('Name', sprintf('태그 %d - TWR 3D 경로 비교', obj.ID));
            plot3(interpolatedTagPositions(:,2), interpolatedTagPositions(:,1), -interpolatedTagPositions(:,3), ...
                  'g-', 'LineWidth', 2, 'DisplayName', '실제 경로');
            hold on;
            plot3(obj.TWRData.EstimatedPosition(:,2), obj.TWRData.EstimatedPosition(:,1), -obj.TWRData.EstimatedPosition(:,3), ...
                  'r--', 'LineWidth', 2, 'DisplayName', 'TWR 추정 경로');
            plot3(anchorPositions(:,2), anchorPositions(:,1), -anchorPositions(:,3), ...
                  'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b', 'DisplayName', 'UWB 앵커');
            
            % 축 범위 설정
            xlim([0 15]);
            ylim([0 15]);
            zlim([0 5]);
            
            % 그래프 설정
            grid on;
            xlabel('동쪽 (m)'); ylabel('북쪽 (m)'); zlabel('높이 (m)');
            title(sprintf('태그 %d - TWR 기반 UAV 위치추적 결과', obj.ID));
            legend('show');
            axis equal;
        end
        
        function stats = getStats(obj)
            % 위치 추정 통계 반환
            %
            % 출력:
            %   stats - 통계 정보 구조체
            
            stats = struct();
            
            if ~isempty(obj.PositionErrors)
                stats.MeanError = mean(obj.PositionErrors);
                stats.MaxError = max(obj.PositionErrors);
                stats.MinError = min(obj.PositionErrors);
                stats.StdError = std(obj.PositionErrors);
                stats.Count = length(obj.PositionErrors);
            else
                stats.MeanError = NaN;
                stats.MaxError = NaN;
                stats.MinError = NaN;
                stats.StdError = NaN;
                stats.Count = 0;
            end
            
            % 축별 오차 정보 제거 - 전체 오차만 사용
        end
        
        function plotCombinedResults(obj, otherTags, anchorPositions)
            % 여러 태그의 결과를 하나의 그래프에 표시
            %
            % 입력:
            %   otherTags       - 다른 태그 객체 배열
            %   anchorPositions - 앵커 위치 행렬
            
            figure('Name', '모든 태그 TWR 3D 경로 비교');
            hold on;
            
            % 현재 태그 결과 표시
            if ~isempty(obj.TWRData.EstimatedTime) && ~isempty(obj.TWRData.EstimatedPosition)
                % 추정 시간에 해당하는 실제 위치 계산
                interpolatedTagPositions = zeros(length(obj.TWRData.EstimatedTime), 3);
                for i = 1:length(obj.TWRData.EstimatedTime)
                    est_time = obj.TWRData.EstimatedTime(i);
                    [~, idx] = min(abs(obj.TWRData.Time - est_time));
                    if ~isempty(idx) && idx <= size(obj.TWRData.TagPosition, 1)
                        interpolatedTagPositions(i, :) = obj.TWRData.TagPosition(idx, :);
                    end
                end
                
                % 실제 경로와 추정 경로 그리기
                plot3(interpolatedTagPositions(:,2), interpolatedTagPositions(:,1), -interpolatedTagPositions(:,3), ...
                      'Color', obj.Color, 'LineStyle', '-', 'LineWidth', 2, ...
                      'DisplayName', sprintf('태그 %d 실제 경로', obj.ID));
                
                plot3(obj.TWRData.EstimatedPosition(:,2), obj.TWRData.EstimatedPosition(:,1), -obj.TWRData.EstimatedPosition(:,3), ...
                      'Color', obj.Color, 'LineStyle', '--', 'LineWidth', 1.5, ...
                      'DisplayName', sprintf('태그 %d TWR 추정 경로', obj.ID));
            end
            
            % 다른 태그 결과 표시
            for i = 1:length(otherTags)
                tag = otherTags(i);
                
                if ~isempty(tag.TWRData.EstimatedTime) && ~isempty(tag.TWRData.EstimatedPosition)
                    % 추정 시간에 해당하는 실제 위치 계산
                    interpolatedTagPositions = zeros(length(tag.TWRData.EstimatedTime), 3);
                    for j = 1:length(tag.TWRData.EstimatedTime)
                        est_time = tag.TWRData.EstimatedTime(j);
                        [~, idx] = min(abs(tag.TWRData.Time - est_time));
                        if ~isempty(idx) && idx <= size(tag.TWRData.TagPosition, 1)
                            interpolatedTagPositions(j, :) = tag.TWRData.TagPosition(idx, :);
                        end
                    end
                    
                    % 실제 경로와 추정 경로 그리기
                    plot3(interpolatedTagPositions(:,2), interpolatedTagPositions(:,1), -interpolatedTagPositions(:,3), ...
                          'Color', tag.Color, 'LineStyle', '-', 'LineWidth', 2, ...
                          'DisplayName', sprintf('태그 %d 실제 경로', tag.ID));
                    
                    plot3(tag.TWRData.EstimatedPosition(:,2), tag.TWRData.EstimatedPosition(:,1), -tag.TWRData.EstimatedPosition(:,3), ...
                          'Color', tag.Color, 'LineStyle', '--', 'LineWidth', 1.5, ...
                          'DisplayName', sprintf('태그 %d TWR 추정 경로', tag.ID));
                end
            end
            
            % 앵커 위치 표시
            plot3(anchorPositions(:,2), anchorPositions(:,1), -anchorPositions(:,3), ...
                  'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k', 'DisplayName', 'UWB 앵커');
            
            % 그래프 설정
            grid on;
            xlabel('동쪽 (m)'); ylabel('북쪽 (m)'); zlabel('높이 (m)');
            title('TWR 기반 다중 UAV 위치추적 결과');
            legend('show', 'Location', 'best');
            axis equal;
            xlim([0 15]);
            ylim([0 15]);
            zlim([0 5]);
            view(45, 30);
        end
    end
end