classdef UAVTag < handle
    % UAVTag UWB 태그가 장착된 UAV 드론 클래스
    
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
        TDoAData            % TDoA 데이터 저장 구조체
        CurrentReceptions   % 현재 신호 수신 상태
        LastBlinkID         % 마지막 블링크 ID
        PositionErrors      % 위치 추정 오차 기록
        Color               % 시각화를 위한 색상

        % TWR-specific properties
        TWRData             % TWR 데이터 저장 구조체
        MeasuredDistances   % 각 앵커와의 측정된 거리
        RequestInterval     % TWR 요청 간격
        LastRequestTime     % 마지막 TWR 요청 시간
        TWRMode             % TWR 모드 활성화 여부

    end
    
    methods
        function obj = UAVTag(id, scenario, initialPosition, waypoints, timeOfArrival)
            % UAVTag 생성자.
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
            obj.LastBlinkID = -1;
            
            % 드론 색상 설정 (ID에 따라 다른 색상)
            colorIndex = mod(id-1, 6) + 1;
            colorMap = [1 0 0;     % Red
                        0 1 0;     % Green
                        0 0 1;     % Blue
                        0 0 0;     % Black
                        1 0 1;      
                        0 1 1];    
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
            
            
            % UWB 센서 모델 생성 및 설정
            obj.SensorModel = uavUWB(id, 'tx');
            obj.SensorModel.configureUWB();
            
            % 각 드론마다 블링크 시간을 약간 다르게 설정 (충돌 방지)
            obj.SensorModel.BlinkInterval = 0.05 + (id * 0.005); % 약간씩 다른 간격
            obj.SensorModel.LastBlinkTime = -0.1 - (id * 0.05);  % 다른 시작 시간
            
            % UWB 센서 장착
            obj.UWBSensor = uavSensor('UWB', obj.Platform, obj.SensorModel);
            
            % 드론 시각화 (ID에 따라 다른 색상)
            updateMesh(obj.Platform, "quadrotor", {0.8}, obj.Color, eul2tform([0 0 pi]));
            
            % TDoA 데이터 구조체 초기화
            obj.TDoAData = struct();
            obj.TDoAData.Time = [];
            obj.TDoAData.TagPosition = [];
            obj.TDoAData.EstimatedTime = [];
            obj.TDoAData.EstimatedPosition = [];
            obj.TDoAData.AnchorReceptions = cell(5, 1);
            obj.TDoAData.TDOA = [];
            
            % 칼만 필터 초기화
            obj.KalmanState = [initialPosition'; 0; 0; 0]; % 위치와 속도
            obj.KalmanCovariance = diag([10, 10, 10, 1, 1, 1]); % 초기 불확실성
            
            % 현재 수신 상태 초기화
            obj.CurrentReceptions = struct('ArrivalTime', zeros(5, 1), 'Valid', false(5, 1));
            
            % 오차 기록 초기화
            obj.PositionErrors = [];
            
            % 초기화 완료 메시지
            fprintf('태그 드론 %d 초기화 완료: 위치 [%.2f, %.2f, %.2f]\n', ...
                id, initialPosition(1), initialPosition(2), initialPosition(3));
        end
        
        function processStep(obj, t, anchorModels, anchorPositions, referenceAnchorIdx)
            % 시뮬레이션의 한 스텝 처리
            %
            % 입력:
            %   t                  - 현재 시뮬레이션 시간 (초)
            %   anchorModels       - 앵커 UWB 센서 모델 셀 배열
            %   anchorPositions    - 앵커 위치 행렬 (각 행은 [x, y, z])
            %   referenceAnchorIdx - 기준 앵커 인덱스 (TDoA 계산용)
            
            % 현재 드론 위치 가져오기
            [txPos, ~, ~, ~, ~] = lookupPose(obj.Platform.Trajectory, t);
            obj.TDoAData.Time(end+1) = t;
            obj.TDoAData.TagPosition(end+1, :) = txPos;
            
            % UWB 블링크 전송 시뮬레이션
            [isTransmitting, blinkData] = obj.SensorModel.processTransmission(obj.Platform, t);
            
            if isTransmitting
                % 블링크 전송 중일 때만 처리
                fprintf('태그 %d 블링크: ID=%d, 위치=[%.2f, %.2f, %.2f]\n', ...
                       obj.ID, blinkData.BlinkID, blinkData.Position(1), blinkData.Position(2), blinkData.Position(3));
                
                % 앵커에서의 신호 수신 처리
                validReceptions = obj.processAnchorsReception(t, blinkData, anchorModels, anchorPositions);
                
                % 모든 앵커가 신호를 수신했는지 확인
                if validReceptions == length(anchorModels)
                    % 모든 앵커에서 수신 성공 시 TDoA 계산 및 위치 추정
                    obj.processTDoA(t, txPos, anchorPositions, referenceAnchorIdx);
                else
                    fprintf('태그 %d: 일부 앵커에서 수신 실패 - 유효한 수신: %d/%d\n', ...
                        obj.ID, validReceptions, length(anchorModels));
                end
            end
        end
        
        function validCount = processAnchorsReception(obj, t, blinkData, anchorModels, anchorPositions)
            % 앵커에서의 신호 수신 처리
            %
            % 입력:
            %   t               - 현재 시간 (초)
            %   blinkData       - 전송된 블링크 데이터
            %   anchorModels    - 앵커 UWB 센서 모델 셀 배열
            %   anchorPositions - 앵커 위치 행렬
            %
            % 출력:
            %   validCount - 유효한 수신이 이루어진 앵커 수
            
            validCount = 0;
            
            for i = 1:length(anchorModels)
                try
                    % 해당 앵커의 센서 모델 사용
                    sensorModel = anchorModels{i};
                    receivedSignal = sensorModel.simulateReception(blinkData.Position, blinkData, anchorPositions(i,:), t);
                    
                    % 신호 검출 임계값 확인
                    if receivedSignal.RSSI > sensorModel.DetectionThreshold
                        % 신호 검출 성공
                        obj.CurrentReceptions.ArrivalTime(i) = receivedSignal.ArrivalTime;
                        obj.CurrentReceptions.Valid(i) = true;
                        validCount = validCount + 1;
                        
                        % 전체 시뮬레이션 데이터에 저장
                        newReception = struct('Time', t, ...
                            'ArrivalTime', receivedSignal.ArrivalTime, ...
                            'RSSI', receivedSignal.RSSI, ...
                            'BlinkID', blinkData.BlinkID);
                        
                        % NLoS 상태 저장 (있는 경우)
                        if isfield(receivedSignal, 'IsNLoS')
                            newReception.IsNLoS = receivedSignal.IsNLoS;
                        else
                            newReception.IsNLoS = false;
                        end
                        
                        % 앵커 수신 데이터 저장
                        if isempty(obj.TDoAData.AnchorReceptions{i})
                            obj.TDoAData.AnchorReceptions{i} = newReception;
                        else
                            obj.TDoAData.AnchorReceptions{i}(end+1) = newReception;
                        end
                    else
                        % 신호 검출 실패
                        obj.CurrentReceptions.Valid(i) = false;
                        fprintf('앵커 %d: 태그 %d 신호 검출 실패 (RSSI: %.2f dBm, 임계값: %.2f dBm)\n', ...
                            i, obj.ID, receivedSignal.RSSI, sensorModel.DetectionThreshold);
                    end
                catch e
                    % 예외 처리
                    fprintf('앵커 %d: 태그 %d 신호 수신 오류: %s\n', i, obj.ID, e.message);
                    obj.CurrentReceptions.Valid(i) = false;
                end
            end
        end
        
        function processTDoA(obj, t, txPos, anchorPositions, referenceAnchorIdx)
            % TDoA 계산 및 위치 추정 처리
            %
            % 입력:
            %   t                  - 현재 시간 (초)
            %   txPos              - 실제 태그 위치 (검증용)
            %   anchorPositions    - 앵커 위치 행렬
            %   referenceAnchorIdx - 기준 앵커 인덱스
            
            c = physconst('LightSpeed'); % 빛의 속도
            
            % TDoA 계산 (기준 앵커와의 시간 차이)
            arrivalTimes = obj.CurrentReceptions.ArrivalTime;
            tdoa_values = arrivalTimes - arrivalTimes(referenceAnchorIdx);
            obj.TDoAData.TDOA(end+1, :) = tdoa_values';
            
            try
                % 위치 추정 (비선형 최소제곱법)
                estimated_pos = obj.estimatePositionTDOA(anchorPositions, tdoa_values, c, referenceAnchorIdx);
                
                % NaN이 없는지 확인
                if ~any(isnan(estimated_pos))
                    % 칼만 필터 적용하여 위치 추정 개선
                    obj.applyKalmanFilter(estimated_pos);
                    
                    % 위치 추정 오차 계산
                    error = norm(txPos - estimated_pos);
                    obj.PositionErrors(end+1) = error;
                    
                    % 각 축별 오차 계산 및 출력
                    x_error = abs(txPos(1) - estimated_pos(1));
                    y_error = abs(txPos(2) - estimated_pos(2));
                    z_error = abs(txPos(3) - estimated_pos(3));
                    
                    fprintf('태그 %d 위치 추정 - 시간: %.2f초\n', obj.ID, t);
                    fprintf('  실제 위치: [%.2f, %.2f, %.2f]\n', txPos(1), txPos(2), txPos(3));
                    fprintf('  추정 위치: [%.2f, %.2f, %.2f]\n', estimated_pos(1), estimated_pos(2), estimated_pos(3));
                    fprintf('  X축 오차: %.2f m, Y축 오차: %.2f m, Z축 오차: %.2f m\n', x_error, y_error, z_error);
                    fprintf('  전체 오차: %.2f m\n', error);
                else
                    fprintf('태그 %d 위치 추정 결과에 NaN 값 포함 - 시간 %.2f초\n', obj.ID, t);
                end
            catch e
                fprintf('태그 %d 위치 추정 오류 - 시간 %.2f초: %s\n', obj.ID, t, e.message);
            end
        end
        
        function applyKalmanFilter(obj, raw_position)
            % 칼만 필터 적용
            %
            % 입력:
            %   raw_position - TDoA로 추정된 원시 위치 [x, y, z]
            
            % 상태 전이 행렬 (등속도 모델)
            dt = obj.SensorModel.BlinkInterval; % 업데이트 간 시간 단계
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
            tdoa_raw_position = raw_position';
            
            % 칼만 이득 계산
            K = obj.KalmanCovariance * H' / (H * obj.KalmanCovariance * H' + R);
            
            % 측정값으로 상태 업데이트
            measurement_residual = tdoa_raw_position - H * obj.KalmanState;
            obj.KalmanState = obj.KalmanState + K * measurement_residual;
            
            % 공분산 업데이트
            obj.KalmanCovariance = (eye(6) - K * H) * obj.KalmanCovariance;
            
            % 필터링된 위치 추정값
            estimated_pos = obj.KalmanState(1:3)';
            
            % 유효한 추정 결과 저장
            obj.TDoAData.EstimatedTime(end+1) = obj.TDoAData.Time(end);
            obj.TDoAData.EstimatedPosition(end+1, :) = estimated_pos;
        end
        
        function position = estimatePositionTDOA(obj, anchorPositions, tdoa_values, c, referenceAnchorIdx)
            % TDoA 위치 추정
            %
            % 입력:
            %   anchorPositions    - 앵커 위치 행렬
            %   tdoa_values        - TDoA 측정값 벡터
            %   c                  - 신호 전파 속도 (m/s)
            %   referenceAnchorIdx - 기준 앵커 인덱스
            %
            % 출력:
            %   position - 추정된 위치 [x, y, z]
            
            % 거리 차이로 변환 (시간 차이 * 빛의 속도)
            range_diff = tdoa_values * c;
            
            % 초기 추측값 - NED 좌표계에서 앵커들의 중심점
            initial_guess = mean(anchorPositions, 1);
            
            % 비선형 최소제곱법으로 위치 추정
            % NED 좌표계에서 z는 아래 방향으로 음수이므로 상한과 하한을 조정
            lb = [0, 0, -5];    % 하한값 (x, y, z) - 천장
            ub = [15, 15, 0];   % 상한값 (x, y, z) - 바닥
            options = optimoptions('lsqnonlin', 'Display', 'off', 'MaxIterations', 100);
            position = lsqnonlin(@tdoa_cost_function, initial_guess, lb, ub, options);
            
            % 비용 함수 정의 (내부 함수)
            function residuals = tdoa_cost_function(pos)
                % 각 앵커와의 거리 계산
                distances = zeros(size(anchorPositions, 1), 1);
                for i = 1:size(anchorPositions, 1)
                    distances(i) = norm(pos - anchorPositions(i,:));
                end
                
                % 기준 앵커와의 거리 차이 계산
                ref_distance = distances(referenceAnchorIdx);
                theoretical_range_diff = distances - ref_distance;
                
                % 측정값과 이론값 간의 차이 반환
                residuals = theoretical_range_diff - range_diff;
            end
        end
        
        function plotResults(obj, anchorPositions)
            % 시뮬레이션 결과 그래프 출력
            %
            % 입력:
            %   anchorPositions - 앵커 위치 행렬
            
            % 결과가 없으면 종료
            if isempty(obj.TDoAData.EstimatedTime) || isempty(obj.TDoAData.EstimatedPosition)
                warning('태그 %d: 위치 추정 데이터가 없습니다. 그래프를 그릴 수 없습니다.', obj.ID);
                return;
            end
            
            % 각 추정 시간에 해당하는 실제 위치 찾기
            interpolatedTagPositions = zeros(length(obj.TDoAData.EstimatedTime), 3);
            for i = 1:length(obj.TDoAData.EstimatedTime)
                est_time = obj.TDoAData.EstimatedTime(i);
                % 가장 가까운 시간 인덱스 찾기
                [~, idx] = min(abs(obj.TDoAData.Time - est_time));
                if ~isempty(idx) && idx <= size(obj.TDoAData.TagPosition, 1)
                    interpolatedTagPositions(i, :) = obj.TDoAData.TagPosition(idx, :);
                end
            end
            
            % 그래프 생성
            figure('Name', sprintf('태그 %d - 위치 추적 결과', obj.ID));
            
            % X, Y, Z 좌표 비교
            subplot(2,2,1);
            plot(obj.TDoAData.EstimatedTime, interpolatedTagPositions(:,1), 'g-', ...
                 obj.TDoAData.EstimatedTime, obj.TDoAData.EstimatedPosition(:,1), 'r--');
            title(sprintf('태그 %d - 북쪽(N) 좌표 비교', obj.ID));
            xlabel('시간 (초)'); ylabel('N 좌표 (m)');
            legend('실제 위치', 'TDoA 추정 위치');
            grid on;
            
            subplot(2,2,2);
            plot(obj.TDoAData.EstimatedTime, interpolatedTagPositions(:,2), 'g-', ...
                 obj.TDoAData.EstimatedTime, obj.TDoAData.EstimatedPosition(:,2), 'r--');
            title(sprintf('태그 %d - 동쪽(E) 좌표 비교', obj.ID));
            xlabel('시간 (초)'); ylabel('E 좌표 (m)');
            legend('실제 위치', 'TDoA 추정 위치');
            grid on;
            
            subplot(2,2,3);
            plot(obj.TDoAData.EstimatedTime, -interpolatedTagPositions(:,3), 'g-', ...
                 obj.TDoAData.EstimatedTime, -obj.TDoAData.EstimatedPosition(:,3), 'r--');
            title(sprintf('태그 %d - 높이(Height) 좌표 비교', obj.ID));
            xlabel('시간 (초)'); ylabel('높이 (m)');
            legend('실제 위치', 'TDoA 추정 위치');
            grid on;
            
            subplot(2,2,4);
            plot(obj.TDoAData.EstimatedTime, obj.PositionErrors, 'b-');
            title(sprintf('태그 %d - 위치 추정 오차', obj.ID));
            xlabel('시간 (초)'); ylabel('오차 (m)');
            grid on;
            
            % 3D 경로 비교 (별도 그림)
            figure('Name', sprintf('태그 %d - 3D 경로 비교', obj.ID));
            plot3(interpolatedTagPositions(:,2), interpolatedTagPositions(:,1), -interpolatedTagPositions(:,3), ...
                  'g-', 'LineWidth', 2, 'DisplayName', '실제 경로');
            hold on;
            plot3(obj.TDoAData.EstimatedPosition(:,2), obj.TDoAData.EstimatedPosition(:,1), -obj.TDoAData.EstimatedPosition(:,3), ...
                  'r--', 'LineWidth', 2, 'DisplayName', 'TDoA 추정 경로');
            plot3(anchorPositions(:,2), anchorPositions(:,1), -anchorPositions(:,3), ...
                  'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b', 'DisplayName', 'UWB 앵커');
            
            % 축 범위 설정
            xlim([0 15]);
            ylim([0 15]);
            zlim([0 5]);
            
            % 그래프 설정
            grid on;
            xlabel('동쪽 (m)'); ylabel('북쪽 (m)'); zlabel('높이 (m)');
            title(sprintf('태그 %d - TDoA 기반 UAV 위치추적 결과', obj.ID));
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
            
            % 축별 오차 계산 (있는 경우)
            if ~isempty(obj.TDoAData.EstimatedTime)
                % 추정 시간에 해당하는 실제 위치 찾기
                interpolatedTagPositions = zeros(length(obj.TDoAData.EstimatedTime), 3);
                for i = 1:length(obj.TDoAData.EstimatedTime)
                    est_time = obj.TDoAData.EstimatedTime(i);
                    [~, idx] = min(abs(obj.TDoAData.Time - est_time));
                    if ~isempty(idx) && idx <= size(obj.TDoAData.TagPosition, 1)
                        interpolatedTagPositions(i, :) = obj.TDoAData.TagPosition(idx, :);
                    end
                end
                
                % 축별 오차
                xErrors = abs(interpolatedTagPositions(:,1) - obj.TDoAData.EstimatedPosition(:,1));
                yErrors = abs(interpolatedTagPositions(:,2) - obj.TDoAData.EstimatedPosition(:,2));
                zErrors = abs(interpolatedTagPositions(:,3) - obj.TDoAData.EstimatedPosition(:,3));
                
                stats.MeanXError = mean(xErrors);
                stats.MeanYError = mean(yErrors);
                stats.MeanZError = mean(zErrors);
            else
                stats.MeanXError = NaN;
                stats.MeanYError = NaN;
                stats.MeanZError = NaN;
            end
        end
    end
end