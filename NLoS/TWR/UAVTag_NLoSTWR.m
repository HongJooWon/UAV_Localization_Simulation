classdef UAVTag_NLoSTWR < handle
    % UAVTag_ParticleTWR UWB TWR 태그가 장착된 UAV 드론 클래스 (파티클 필터 버전)
    % 
    % 이 클래스는 Two-Way Ranging(TWR) 위치 추정 시스템에서 태그 역할을 하는 드론을 객체화합니다.
    % 드론의 움직임, UWB 신호 전송 및 수신, 파티클 필터 기반 위치 추정 결과 등을 관리합니다.
    % NLoS(Non-Line-of-Sight) 환경 지원 기능 추가됨
    
    properties
        ID                  % 드론의 고유 ID
        Platform            % UAV 플랫폼 객체
        UWBSensor           % UWB 센서 객체
        SensorModel         % UWB 센서 모델
        InitialPosition     % 초기 위치 (NED 좌표계)
        Waypoints           % 경로점
        TimeOfArrival       % 각 경로점 도착 시간
        TWRData             % TWR 데이터 저장 구조체
        MeasuredDistances   % 각 앵커와의 측정된 거리
        RequestInterval     % TWR 요청 간격
        LastRequestTime     % 마지막 TWR 요청 시간
        PositionErrors      % 위치 추정 오차 기록
        Color               % 시각화를 위한 색상
        
        % 파티클 필터 관련 속성
        Particles           % 파티클 집합
        ParticleWeights     % 파티클 가중치
        NumParticles        % 파티클 수
        ParticleFilterActive % 파티클 필터 활성화 여부
    end
    
    methods
        function obj = UAVTag_NLoSTWR(id, scenario, initialPosition, waypoints, timeOfArrival, measurementInterval)
            % UAVTag_ParticleTWR 생성자: 태그 드론 객체를 초기화합니다.
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
            if nargin < 6
                obj.RequestInterval = 0.05; % 기본값 설정
            else
                obj.RequestInterval = measurementInterval;
            end
            obj.SensorModel.RequestInterval = obj.RequestInterval;
            obj.LastRequestTime = -0.1 - (id * 0.01);  % 시작 시간만 차이를 두어 충돌 방지
            
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
            
            % 파티클 필터 초기화
            obj.ParticleFilterActive = false; % 생성자에서는 비활성화 상태로 시작
            
            % 오차 기록 초기화
            obj.PositionErrors = [];
            
            % 초기화 완료 메시지
            fprintf('TWR 태그 드론 %d 초기화 완료: 위치 [%.2f, %.2f, %.2f]\n', ...
                id, initialPosition(1), initialPosition(2), initialPosition(3));
        end
        
        function initParticleFilter(obj, numParticles)
            % 파티클 필터 초기화
            obj.NumParticles = numParticles;
            obj.ParticleFilterActive = true;
            
            % 초기 파티클 생성 (초기 위치 주변에 분포)
            obj.Particles = zeros(numParticles, 3);
            for i = 1:numParticles
                obj.Particles(i, :) = obj.InitialPosition + randn(1, 3) .* [2, 2, 1];
            end
            
            % 초기 가중치는 균등하게 설정
            obj.ParticleWeights = ones(numParticles, 1) / numParticles;
            
            fprintf('태그 %d 파티클 필터 초기화 완료: %d개 파티클\n', obj.ID, numParticles);
        end
        
        function processStep(obj, t, anchorPlatforms, anchorModels, anchorPositions, obstacles)
            % 시뮬레이션의 한 스텝 처리 - NLoS 환경 지원을 위해 obstacles 매개변수 추가
            %
            % 입력:
            %   t                - 현재 시뮬레이션 시간 (초)
            %   anchorPlatforms  - 앵커 플랫폼 배열
            %   anchorModels     - 앵커 UWB 센서 모델 셀 배열
            %   anchorPositions  - 앵커 위치 행렬 (각 행은 [x, y, z])
            %   obstacles        - 장애물 구조체 배열 (NLoS 효과용) - 선택적 파라미터
            
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
                    % obstacles 매개변수를 processTWRRequestResponse에 전달
                    if nargin < 6
                        obj.processTWRRequestResponse(txPos, requestData, currentAnchorIdx, t, anchorPlatforms, anchorModels, anchorPositions);
                    else
                        obj.processTWRRequestResponse(txPos, requestData, currentAnchorIdx, t, anchorPlatforms, anchorModels, anchorPositions, obstacles);
                    end
                end
            end
        end
        
        function processTWRRequestResponse(obj, txPos, requestData, currentAnchorIdx, t, anchorPlatforms, anchorModels, anchorPositions, obstacles)
            % TWR 요청 및 응답 사이클 처리 - NLoS 환경 지원을 위한 obstacles 매개변수 추가
            %
            % 입력:
            %   txPos             - 태그의 현재 위치
            %   requestData       - TWR 요청 데이터
            %   currentAnchorIdx  - 현재 앵커 인덱스
            %   t                 - 현재 시간
            %   anchorPlatforms   - 앵커 플랫폼 배열
            %   anchorModels      - 앵커 센서 모델 배열
            %   anchorPositions   - 앵커 위치 배열
            %   obstacles         - 장애물 구조체 배열 (NLoS 효과용) - 선택적 파라미터
            
            try
                % 전파 지연 계산 (NLoS 고려)
                if nargin < 8 || isempty(obstacles)
                    % obstacles가 제공되지 않은 경우 기본 전파 지연 계산
                    distance = norm(txPos - anchorPositions(currentAnchorIdx,:));
                    propagationDelay = distance / physconst('LightSpeed');
                    isNLoS = false;
                else
                    % NLoS 검사 및 전파 지연 계산
                    [isNLoS, additionalDelay, ~] = obj.SensorModel.checkNLoS(txPos, anchorPositions(currentAnchorIdx,:), obstacles);
                    distance = norm(txPos - anchorPositions(currentAnchorIdx,:));
                    propagationDelay = distance / physconst('LightSpeed') + additionalDelay;
                    
                    if isNLoS
                        fprintf('태그 %d -> 앵커 %d: NLoS 상태 감지됨 (추가 지연: %.2f ns)\n', ...
                            obj.ID, currentAnchorIdx, additionalDelay * 1e9);
                    end
                end
                
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
                
                % 앵커에서 태그로의 응답 전파 계산 (NLoS 고려)
                if nargin < 8 || isempty(obstacles)
                    % obstacles가 제공되지 않은 경우 기본 전파 지연 계산
                    responseArrivalTime = responseTime + propagationDelay;
                else
                    % 응답 경로에 대한 NLoS 검사 (요청 경로와 동일한 결과를 가질 수 있지만 양방향성을 보장하기 위해 다시 계산)
                    [~, returnAdditionalDelay, ~] = obj.SensorModel.checkNLoS(anchorPositions(currentAnchorIdx,:), txPos, obstacles);
                    returnPropagationDelay = distance / physconst('LightSpeed') + returnAdditionalDelay;
                    responseArrivalTime = responseTime + returnPropagationDelay;
                    
                    if returnAdditionalDelay > 0 && returnAdditionalDelay ~= additionalDelay
                        fprintf('앵커 %d -> 태그 %d: 비대칭 NLoS 상태 감지됨 (추가 지연: %.2f ns)\n', ...
                            currentAnchorIdx, obj.ID, returnAdditionalDelay * 1e9);
                    end
                end
                
                % 태그에서 응답 처리
                obj.SensorModel.processTWRResponse(responseData, responseArrivalTime);
                
                % 측정된 거리 저장
                fieldName = ['anchor_', num2str(currentAnchorIdx)];
                if isfield(obj.SensorModel.MeasuredDistances, fieldName)
                    obj.MeasuredDistances(currentAnchorIdx) = obj.SensorModel.MeasuredDistances.(fieldName);
                end
                
                % 모든 앵커와의 거리가 측정되었는지 확인
                if all(obj.MeasuredDistances > 0)
                    % 파티클 필터 활성화 여부에 따라 위치 추정 방법 결정
                    if obj.ParticleFilterActive
                        % 파티클 필터로 위치 추정
                        estimated_pos = obj.estimatePositionWithParticleFilter(anchorPositions, obj.MeasuredDistances, t);
                    else
                        % 기존 방식으로 위치 추정
                        fprintf('파티클 아님\n');
                        estimated_pos = obj.estimatePositionTWR(anchorPositions, obj.MeasuredDistances);
                    end
                    
                    if ~any(isnan(estimated_pos))
                        % 위치 추정 결과 저장
                        obj.TWRData.EstimatedTime(end+1) = t;
                        obj.TWRData.EstimatedPosition(end+1, :) = estimated_pos;
                        
                        % 오차 계산
                        error = norm(txPos - estimated_pos);
                        obj.PositionErrors(end+1) = error;
                        
                        % 결과 출력
                        fprintf('태그 %d TWR 위치 추정 - 시간: %.2f초\n', obj.ID, t);
                        fprintf('  실제 위치: [%.2f, %.2f, %.2f]\n', txPos(1), txPos(2), txPos(3));
                        fprintf('  추정 위치: [%.2f, %.2f, %.2f]\n', estimated_pos(1), estimated_pos(2), estimated_pos(3));
                        fprintf('  전체 오차: %.2f m\n', error);
                        
                        % 다음 사이클을 위해 거리 재설정
                        % obj.MeasuredDistances = zeros(5, 1);
                    end
                end
            catch e
                fprintf('태그 %d TWR 처리 오류: %s\n', obj.ID, e.message);
            end
        end
        
        function estimated_pos = estimatePositionWithParticleFilter(obj, anchorPositions, distances, t)
            % 파티클 필터를 사용한 위치 추정
            %
            % 입력:
            %   anchorPositions - 앵커 위치 행렬
            %   distances       - 각 앵커까지의 측정된 거리
            %   t               - 현재 시간
            %
            % 출력:
            %   estimated_pos   - 추정된 위치 [x, y, z]
            
            % 1. 파티클 예측 (드론 움직임 예측)
            obj.predictParticles(obj.RequestInterval);
            
            % 2. 가중치 업데이트 (TWR 측정값 기반)
            obj.updateWeightsTWR(anchorPositions, distances);
            
            % 3. 리샘플링
            obj.resampleParticles();
            
            % 4. 상태 추정
            topN = ceil(obj.NumParticles * 0.1);
            estimated_pos = obj.estimatePosition();
            
            return;
        end

        function pos = estimatePositionTopN(obj, topN)
            % 상위 N개 파티클만 사용하여 위치 추정
            %
            % 입력:
            %   topN - 사용할 상위 파티클 개수 (기본값: 파티클 총 개수의 10%)
            %
            % 출력:
            %   pos - 추정된 위치 [x, y, z]
            
            if nargin < 2
                topN = ceil(obj.NumParticles * 0.1); % 기본값으로 상위 10% 사용
            end
            
            % 가중치에 따라 파티클 정렬
            [sorted_weights, indices] = sort(obj.ParticleWeights, 'descend');
            
            % 상위 N개만 선택
            top_indices = indices(1:min(topN, length(indices)));
            top_particles = obj.Particles(top_indices, :);
            top_weights = sorted_weights(1:min(topN, length(indices)));
            
            % 정규화된 가중치 계산
            normalized_weights = top_weights / sum(top_weights);
            
            % 가중 평균 계산
            pos = zeros(1, 3);
            for i = 1:3
                pos(i) = sum(top_particles(:, i) .* normalized_weights);
            end
            
            fprintf('태그 %d: 상위 %d개 파티클 평균으로 위치 추정\n', obj.ID, min(topN, length(indices)));
        end
        
        function predictParticles(obj, dt)
            % 파티클 예측 단계
            % 시간 간격 dt 동안의 드론 움직임을 반영
            
            for i = 1:obj.NumParticles
                % 랜덤 워크 모션 모델 (간단한 모델)
                obj.Particles(i, :) = obj.Particles(i, :) + 0.05 * randn(1, 3);
                
                % 공간 제약 적용 (실내 환경 경계)
                obj.Particles(i, 1) = max(0, min(15, obj.Particles(i, 1))); % x 제약
                obj.Particles(i, 2) = max(0, min(15, obj.Particles(i, 2))); % y 제약
                obj.Particles(i, 3) = max(-5, min(0, obj.Particles(i, 3))); % z 제약
            end
        end
        
        function updateWeightsTWR(obj, anchorPositions, distances)
            % TWR 측정을 기반으로 파티클 가중치 업데이트
            %
            % 입력:
            %   anchorPositions - 앵커 위치 행렬
            %   distances       - 각 앵커까지의 측정된 거리
            
            for i = 1:obj.NumParticles
                likelihood = 1;
                
                % 파티클 위치
                particle_pos = obj.Particles(i, :);
                
                % 각 앵커에 대한 거리 우도 계산
                for a = 1:length(distances)
                    if distances(a) > 0 % 측정된 거리가 있는 경우만
                        % 파티클과 앵커 간 이론적 거리 계산
                        theoretical_dist = norm(particle_pos - anchorPositions(a,:));
                        
                        % 측정 거리
                        measured_dist = distances(a);
                        
                        % 오차 계산
                        dist_error = measured_dist - theoretical_dist;
                        
                        % 우도 계산 (가우시안 노이즈 가정)
                        dist_likelihood = exp(-0.5 * (dist_error / 0.1)^2); % 5cm 표준편차 가정
                        likelihood = likelihood * dist_likelihood;
                    end
                end
                
                % 가중치 업데이트
                obj.ParticleWeights(i) = likelihood;
            end
            
            % 가중치 정규화
            if sum(obj.ParticleWeights) > 0
                obj.ParticleWeights = obj.ParticleWeights / sum(obj.ParticleWeights);
            else
                % 모든 가중치가 0이면 균등하게 재설정
                obj.ParticleWeights = ones(obj.NumParticles, 1) / obj.NumParticles;
            end
        end
        
        function resampleParticles(obj)
            % 파티클 리샘플링 처리
            
            % 효과적인 샘플 크기 계산
            Neff = 1 / sum(obj.ParticleWeights.^2);
            
            % 리샘플링 임계값
            resampleThreshold = obj.NumParticles / 2;
            
            if Neff < resampleThreshold
                % 누적 가중치 계산
                cum_weights = cumsum(obj.ParticleWeights);
                
                % 균일 간격 샘플링 시작점
                u1 = rand() / obj.NumParticles;
                
                % 새 파티클 인덱스
                indices = zeros(obj.NumParticles, 1);
                i = 1;
                for j = 1:obj.NumParticles
                    u = u1 + (j-1) / obj.NumParticles;
                    while u > cum_weights(i) && i < obj.NumParticles
                        i = i + 1;
                    end
                    indices(j) = i;
                end
                
                % 파티클 리샘플링
                obj.Particles = obj.Particles(indices, :);
                
                % 가중치 균등 재설정
                obj.ParticleWeights = ones(obj.NumParticles, 1) / obj.NumParticles;
                
                fprintf('태그 %d: 파티클 리샘플링 수행 (Neff = %.2f)\n', obj.ID, Neff);
            end
        end
        
        function pos = estimatePosition(obj)
            % 파티클 가중 평균으로 위치 추정
            pos = zeros(1, 3);
            for i = 1:3
                pos(i) = sum(obj.Particles(:, i) .* obj.ParticleWeights);
            end
        end
        
        function position = estimatePositionTWR(obj, anchorPositions, distances)
            % TWR 위치 추정 (기존 방식)
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
            if obj.ParticleFilterActive
                title(sprintf('태그 %d - 파티클 필터 TWR 기반 UAV 위치추적 결과 (NLoS 적용)', obj.ID));
            else
                title(sprintf('태그 %d - TWR 기반 UAV 위치추적 결과 (NLoS 적용)', obj.ID));
            end
            legend('show');
            axis equal;
        end
        
        function plotParticles(obj, anchorPositions, obstacles)
            % 현재 파티클 분포를 3D로 시각화 (NLoS 환경이 포함된 버전)
            %
            % 입력:
            %   anchorPositions - 앵커 위치 행렬
            %   obstacles       - 장애물 구조체 배열 (선택적)
            
            if ~obj.ParticleFilterActive
                warning('파티클 필터가 활성화되지 않았습니다.');
                return;
            end
            
            figure('Name', sprintf('태그 %d - 파티클 분포 (NLoS 환경)', obj.ID));
            
            % 앵커 위치 표시
            plot3(anchorPositions(:,2), anchorPositions(:,1), -anchorPositions(:,3), ...
                  'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b', 'DisplayName', 'UWB 앵커');
            hold on;
            
            % 장애물 표시 (있는 경우)
            if nargin >= 3 && ~isempty(obstacles)
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
            end
            
            % 파티클 색상 설정 (가중치에 따라)
            max_weight = max(obj.ParticleWeights);
            colors = zeros(obj.NumParticles, 3);
            for i = 1:obj.NumParticles
                % 가중치 정규화 (0.2-1 범위)
                weight_norm = 0.2 + 0.8 * (obj.ParticleWeights(i) / max_weight);
                colors(i, :) = [1, 0, 0] * weight_norm; % 빨간색 계열
            end
            
            % 파티클 표시 (3D 스캐터 플롯)
            scatter3(obj.Particles(:,2), obj.Particles(:,1), -obj.Particles(:,3), 10, colors, 'filled', 'DisplayName', '파티클');
            
            % 추정 위치 표시 (최근 위치)
            if ~isempty(obj.TWRData.EstimatedPosition)
                last_pos = obj.TWRData.EstimatedPosition(end,:);
                plot3(last_pos(2), last_pos(1), -last_pos(3), 'k*', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', '추정 위치');
            end
            
            % 실제 위치 표시 (가능한 경우)
            if ~isempty(obj.TWRData.TagPosition)
                last_real_pos = obj.TWRData.TagPosition(end,:);
                plot3(last_real_pos(2), last_real_pos(1), -last_real_pos(3), 'g*', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', '실제 위치');
            end
            
            % 그래프 설정
            grid on;
            xlabel('동쪽 (m)'); ylabel('북쪽 (m)'); zlabel('높이 (m)');
            title(sprintf('태그 %d - 파티클 필터 TWR 위치 추정 (NLoS 환경)', obj.ID));
            legend('show');
            axis equal;
            xlim([0 15]);
            ylim([0 15]);
            zlim([0 5]);
            view(45, 30);
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

                stats.PositionErrors = obj.PositionErrors;
            else
                stats.MeanError = NaN;
                stats.MaxError = NaN;
                stats.MinError = NaN;
                stats.StdError = NaN;
                stats.Count = 0;
            end
            
            % 축별 오차 계산 (있는 경우)
            if ~isempty(obj.TWRData.EstimatedTime)
                % 추정 시간에 해당하는 실제 위치 찾기
                interpolatedTagPositions = zeros(length(obj.TWRData.EstimatedTime), 3);
                for i = 1:length(obj.TWRData.EstimatedTime)
                    est_time = obj.TWRData.EstimatedTime(i);
                    [~, idx] = min(abs(obj.TWRData.Time - est_time));
                    if ~isempty(idx) && idx <= size(obj.TWRData.TagPosition, 1)
                        interpolatedTagPositions(i, :) = obj.TWRData.TagPosition(idx, :);
                    end
                end
                
                % 축별 오차
                xErrors = abs(interpolatedTagPositions(:,1) - obj.TWRData.EstimatedPosition(:,1));
                yErrors = abs(interpolatedTagPositions(:,2) - obj.TWRData.EstimatedPosition(:,2));
                zErrors = abs(interpolatedTagPositions(:,3) - obj.TWRData.EstimatedPosition(:,3));
                
                stats.MeanXError = mean(xErrors);
                stats.MeanYError = mean(yErrors);
                stats.MeanZError = mean(zErrors);
            else
                stats.MeanXError = NaN;
                stats.MeanYError = NaN;
                stats.MeanZError = NaN;
            end
        end
        
        function plotTimeErrorGraph(obj)
            % 시간 변화에 따른 위치 추정 오차 그래프 생성
            
            if isempty(obj.TWRData.EstimatedTime) || isempty(obj.PositionErrors)
                warning('태그 %d: 위치 추정 오차 데이터가 없습니다. 그래프를 그릴 수 없습니다.', obj.ID);
                return;
            end
            
            % 그래프 생성
            figure('Name', sprintf('태그 %d - 시간에 따른 위치 추정 오차', obj.ID));
            
            % 오차 플로팅
            plot(obj.TWRData.EstimatedTime, obj.PositionErrors, 'b-o', 'LineWidth', 1.5, 'MarkerSize', 4);
            
            % 이동 평균 추가
            hold on;
            window_size = 5; % 이동 평균 윈도우 크기
            if length(obj.PositionErrors) >= window_size
                moving_avg = movmean(obj.PositionErrors, window_size);
                plot(obj.TWRData.EstimatedTime, moving_avg, 'r-', 'LineWidth', 2, 'DisplayName', '이동 평균');
            end
            
            % X축 범위 설정 (0-30초)
            xlim([0 30]);
            
            % Y축 범위 설정 (필요에 따라 조정)
            max_error = max(obj.PositionErrors) * 1.1; % 최대 오차의 110%로 설정
            ylim([0 max_error]);
            
            % 그래프 꾸미기
            xlabel('시간 (초)');
            ylabel('위치 추정 오차 (m)');
            title(sprintf('태그 %d - 시간에 따른 위치 추정 오차 (NLoS 환경)', obj.ID));
            grid on;
            
            if length(obj.PositionErrors) >= window_size
                legend('실시간 오차', '이동 평균 오차', 'Location', 'best');
            end
            
            % 시간 구간 표시 (5초 간격)
            xticks(0:5:30);
        end
        
        function plotCombinedResults(obj, otherTags, anchorPositions, obstacles)
            % 여러 태그의 결과를 하나의 그래프에 표시
            %
            % 입력:
            %   otherTags       - 다른 태그 객체 배열
            %   anchorPositions - 앵커 위치 행렬
            %   obstacles       - 장애물 구조체 배열 (선택적)
            
            figure('Name', '모든 태그 TWR 3D 경로 비교 (NLoS 환경)');
            hold on;
            
            % 장애물 표시 (있는 경우)
            if nargin >= 4 && ~isempty(obstacles)
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
            end
            
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
            title('파티클 필터 TWR 기반 다중 UAV 위치추적 결과 (NLoS 환경)');
            legend('show', 'Location', 'best');
            axis equal;
            xlim([0 15]);
            ylim([0 15]);
            zlim([0 5]);
            view(45, 30);
        end
    end
end