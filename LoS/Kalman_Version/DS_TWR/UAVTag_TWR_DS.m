classdef UAVTag_TWR_DS < handle
    % UAVTag_TWR_DS UWB DS-TWR 태그가 장착된 UAV 드론 클래스
    % 
    % 이 클래스는 Double-Sided Two-Way Ranging(DS-TWR) 위치 추정 시스템에서 
    % 태그 역할을 하는 드론을 객체화합니다.
    % 드론의 움직임, UWB 신호 전송 및 수신, 위치 추정 결과 등을 관리합니다.
    
    properties
        ID                  % 드론의 고유 ID
        Platform            % UAV 플랫폼 객체
        UWBSensor           % UWB 센서 객체
        SensorModel         % UWB 센서 모델
        InitialPosition     % 초기 위치 (NED 좌표계)
        Waypoints           % 경로점
        TimeOfArrival       % 각 경로점 도착 시간
        FilterState         % 필터 상태 (위치만 포함)
        FilterAlpha         % 필터 평활화 계수 (0-1 사이의 값)
        TWRData             % TWR 데이터 저장 구조체
        MeasuredDistances   % 각 앵커와의 측정된 거리
        RequestInterval     % TWR 요청 간격
        LastRequestTime     % 마지막 TWR 요청 시간
        PositionErrors      % 위치 추정 오차 기록
        Color               % 시각화를 위한 색상
        
        % DS-TWR 관련 추가 속성
        PendingDSTWR        % DS-TWR 진행 중인 앵커 정보
        FirstRoundTimes     % 첫 번째 왕복의 시간 정보
        LastProcessedAnchors % 마지막으로 처리된 앵커 인덱스
        ProcessDelay        % 두 번째 요청 전송 전 대기 시간 (초)
        ShowDebug           % 디버그 메시지 표시 여부
    end
    
    methods
        function obj = UAVTag_TWR_DS(id, scenario, initialPosition, waypoints, timeOfArrival)
            % UAVTag_TWR_DS 생성자: 태그 드론 객체를 초기화합니다.
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
            
            % UWB 센서 모델 생성 및 설정 - DS-TWR 모드
            obj.SensorModel = uavUWB(id, 'txrx', true); % txrx 모드와 TWR 활성화
            obj.SensorModel.configureUWB();
            
            % TWR 요청 간격 설정 - 각 드론마다 약간 다르게 (충돌 방지)
            obj.RequestInterval = 0.05 + (id * 0.001); % 드론마다 약간 다른 간격
            obj.SensorModel.RequestInterval = obj.RequestInterval;
            obj.LastRequestTime = -0.1 - (id * 0.01);  % 다른 시작 시간
            
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
            
            % 필터 초기화 (평활화 필터)
            obj.FilterState = initialPosition'; % 초기 위치
            obj.FilterAlpha = 0.2; % 평활화 계수 (0에 가까울수록 필터링 강함)
            
            % 오차 기록 초기화
            obj.PositionErrors = [];
            
            % DS-TWR 관련 초기화
            obj.PendingDSTWR = struct('AnchorID', {}, 'Stage', {}, 'StartTime', {}, 'Timestamp1', {}, 'Timestamp2', {}, 'Timestamp3', {});
            obj.FirstRoundTimes = struct('AnchorID', {}, 'T1', {}, 'T2', {}, 'T3', {}, 'T4', {});
            obj.LastProcessedAnchors = zeros(5, 1); % 각 앵커별 마지막 처리 시간
            obj.ProcessDelay = 0.01; % 10ms 대기 (두 번째 요청 전)
            obj.ShowDebug = true; % 디버그 메시지 표시
            
            % 초기화 완료 메시지
            fprintf('DS-TWR 태그 드론 %d 초기화 완료: 위치 [%.2f, %.2f, %.2f]\n', ...
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
            
            % 현재 처리할 앵커 결정 (라운드 로빈)
            currentAnchorIdx = mod(floor(t / 0.1), 5) + 1;  % 0.1초마다 다른 앵커
            
            % 진행 중인 DS-TWR 프로세스가 있는지 확인
            pendingIdx = -1;
            for i = 1:length(obj.PendingDSTWR)
                if obj.PendingDSTWR(i).AnchorID == currentAnchorIdx
                    pendingIdx = i;
                    break;
                end
            end
            
            % 처리 중이지 않은 앵커에 대해 새 DS-TWR 첫 번째 단계 시작
            if pendingIdx == -1 && t - obj.LastRequestTime >= 0.1
                % DS-TWR 첫 번째 요청 전송
                fprintf('태그 %d - 앵커 %d DS-TWR 첫 번째 요청 시작 (t=%.3f)\n', obj.ID, currentAnchorIdx, t);
                [isSent, requestData] = obj.SensorModel.sendTWRRequest(obj.Platform, currentAnchorIdx, t);
                
                if isSent
                    obj.LastRequestTime = t;
                    
                    % DS-TWR 첫 번째 단계 시작
                    newEntry = struct('AnchorID', currentAnchorIdx, 'Stage', 1, 'StartTime', t, 'Timestamp1', t, 'Timestamp2', 0, 'Timestamp3', 0);
                    
                    % PendingDSTWR에 추가
                    if isempty(fieldnames(obj.PendingDSTWR)) || isempty(obj.PendingDSTWR)
                        obj.PendingDSTWR = newEntry;
                    else
                        obj.PendingDSTWR(end+1) = newEntry;
                    end
                    
                    % 첫 번째 요청의 전파 지연 계산
                    distance = norm(txPos - anchorPositions(currentAnchorIdx,:));
                    propagationDelay = distance / physconst('LightSpeed');
                    
                    % 앵커에서의 요청 도착 시간 (T2)
                    requestArrivalTime = t + propagationDelay;
                    
                    % 앵커의 처리 지연
                    anchorProcessingDelay = 0.001;
                    if isfield(anchorModels{currentAnchorIdx}, 'ProcessingDelay')
                        anchorProcessingDelay = anchorModels{currentAnchorIdx}.ProcessingDelay;
                    end
                    responseTime = requestArrivalTime + anchorProcessingDelay;
                    
                    % 앵커 응답 생성 (T3)
                    [isSent, responseData] = anchorModels{currentAnchorIdx}.sendTWRResponse(anchorPlatforms(currentAnchorIdx), requestData, responseTime);
                    
                    if isSent
                        % 태그에서의 응답 도착 시간 (T4)
                        responseArrivalTime = responseTime + propagationDelay;
                        
                        % 첫 번째 왕복 시간 저장
                        newRound = struct('AnchorID', currentAnchorIdx, 'T1', t, 'T2', requestArrivalTime, 'T3', responseTime, 'T4', responseArrivalTime);
                        
                        % FirstRoundTimes 업데이트
                        if isempty(fieldnames(obj.FirstRoundTimes)) || isempty(obj.FirstRoundTimes)
                            obj.FirstRoundTimes = newRound;
                        else
                            found = false;
                            for i = 1:length(obj.FirstRoundTimes)
                                if obj.FirstRoundTimes(i).AnchorID == currentAnchorIdx
                                    obj.FirstRoundTimes(i) = newRound;
                                    found = true;
                                    break;
                                end
                            end
                            if ~found
                                obj.FirstRoundTimes(end+1) = newRound;
                            end
                        end
                        
                        % PendingDSTWR 업데이트 - 두 번째 단계 준비
                        for i = 1:length(obj.PendingDSTWR)
                            if obj.PendingDSTWR(i).AnchorID == currentAnchorIdx
                                obj.PendingDSTWR(i).Timestamp2 = responseArrivalTime;
                                fprintf('태그 %d - 앵커 %d DS-TWR 첫 번째 요청 완료 (t=%.3f)\n', obj.ID, currentAnchorIdx, responseArrivalTime);
                                break;
                            end
                        end
                    else
                        fprintf('태그 %d - 앵커 %d DS-TWR 첫 번째 응답 실패\n', obj.ID, currentAnchorIdx);
                    end
                else
                    fprintf('태그 %d - 앵커 %d DS-TWR 첫 번째 요청 실패\n', obj.ID, currentAnchorIdx);
                end
            end
            
            % 첫 번째 단계가 완료된 앵커들에 대해 두 번째 단계 처리
            for i = length(obj.PendingDSTWR):-1:1
                if obj.PendingDSTWR(i).Stage == 1 && obj.PendingDSTWR(i).Timestamp2 > 0 && t - obj.PendingDSTWR(i).Timestamp2 >= 0.02
                    anchorIdx = obj.PendingDSTWR(i).AnchorID;
                    
                    % 두 번째 요청 전송 시작
                    fprintf('태그 %d - 앵커 %d DS-TWR 두 번째 요청 시작 (t=%.3f)\n', obj.ID, anchorIdx, t);
                    [isSent, requestData] = obj.SensorModel.sendTWRRequest(obj.Platform, anchorIdx, t);
                    
                    if isSent
                        % 두 번째 단계로 업데이트
                        obj.PendingDSTWR(i).Stage = 2;
                        obj.PendingDSTWR(i).Timestamp3 = t;
                        
                        % 두 번째 요청의 전파 지연 계산
                        distance = norm(txPos - anchorPositions(anchorIdx,:));
                        propagationDelay = distance / physconst('LightSpeed');
                        
                        % 앵커에서의 요청 도착 시간 (T6)
                        requestArrivalTime = t + propagationDelay;
                        
                        % 앵커의 처리 지연
                        anchorProcessingDelay = 0.001;
                        if isfield(anchorModels{anchorIdx}, 'ProcessingDelay')
                            anchorProcessingDelay = anchorModels{anchorIdx}.ProcessingDelay;
                        end
                        responseTime = requestArrivalTime + anchorProcessingDelay;
                        
                        % 앵커 응답 생성 (T7)
                        [isSent, responseData] = anchorModels{anchorIdx}.sendTWRResponse(anchorPlatforms(anchorIdx), requestData, responseTime);
                        
                        if isSent
                            % 태그에서의 응답 도착 시간 (T8)
                            responseArrivalTime = responseTime + propagationDelay;
                            
                            % 첫 번째 왕복 시간 찾기
                            firstRoundIdx = -1;
                            for j = 1:length(obj.FirstRoundTimes)
                                if obj.FirstRoundTimes(j).AnchorID == anchorIdx
                                    firstRoundIdx = j;
                                    break;
                                end
                            end
                            
                            if firstRoundIdx > 0
                                % 첫 번째 왕복의 시간 정보
                                T1 = obj.FirstRoundTimes(firstRoundIdx).T1;
                                T2 = obj.FirstRoundTimes(firstRoundIdx).T2;
                                T3 = obj.FirstRoundTimes(firstRoundIdx).T3;
                                T4 = obj.FirstRoundTimes(firstRoundIdx).T4;
                                
                                % 두 번째 왕복의 시간 정보
                                T5 = t;
                                T6 = requestArrivalTime;
                                T7 = responseTime;
                                T8 = responseArrivalTime;
                                
                                % DS-TWR 거리 계산식
                                % 왕복 시간 계산
                                roundTrip1 = T4 - T1; % 첫 번째 왕복
                                roundTrip2 = T8 - T5; % 두 번째 왕복
                                
                                % 응답 시간 계산
                                replyTime1 = T3 - T2; % 첫 번째 응답 처리 시간
                                replyTime2 = T7 - T6; % 두 번째 응답 처리 시간
                                
                                % DS-TWR 공식으로 왕복 시간 계산 (크리스탈 오차 상쇄)
                                tof = (roundTrip1 * roundTrip2 - replyTime1 * replyTime2) / (roundTrip1 + roundTrip2 + replyTime1 + replyTime2);
                                
                                % 거리 계산
                                distance = tof * physconst('LightSpeed');
                                
                                % 측정된 거리 저장
                                obj.MeasuredDistances(anchorIdx) = distance;
                                
                                fprintf('태그 %d - 앵커 %d DS-TWR 두 번째 요청 완료, 거리: %.3f m (실제: %.3f m)\n', ...
                                        obj.ID, anchorIdx, distance, norm(txPos - anchorPositions(anchorIdx,:)));
                                
                                % 처리 완료된 DS-TWR 요청 제거
                                obj.PendingDSTWR(i) = [];
                            else
                                fprintf('태그 %d - 앵커 %d DS-TWR 첫 번째 왕복 정보 없음\n', obj.ID, anchorIdx);
                                obj.PendingDSTWR(i) = [];
                            end
                        else
                            fprintf('태그 %d - 앵커 %d DS-TWR 두 번째 응답 실패\n', obj.ID, anchorIdx);
                            obj.PendingDSTWR(i) = [];
                        end
                    else
                        fprintf('태그 %d - 앵커 %d DS-TWR 두 번째 요청 실패\n', obj.ID, anchorIdx);
                        obj.PendingDSTWR(i) = [];
                    end
                end
                
                % 타임아웃 처리
                if t - obj.PendingDSTWR(i).StartTime > 0.5
                    fprintf('태그 %d - 앵커 %d DS-TWR 요청 타임아웃\n', obj.ID, obj.PendingDSTWR(i).AnchorID);
                    obj.PendingDSTWR(i) = [];
                end
            end
            
            % 모든 앵커와의 거리가 측정되었는지 확인
            if all(obj.MeasuredDistances > 0)
                % Multilateration으로 위치 추정
                estimated_pos = obj.estimatePositionTWR(anchorPositions, obj.MeasuredDistances);
                
                if ~any(isnan(estimated_pos))
                    % 필터링 적용
                    filtered_pos = obj.applySmoothing(estimated_pos);
                    
                    % 위치 추정 결과 저장
                    obj.TWRData.EstimatedTime(end+1) = t;
                    obj.TWRData.EstimatedPosition(end+1, :) = filtered_pos';
                    
                    % 오차 계산
                    error = norm(txPos - filtered_pos');
                    obj.PositionErrors(end+1) = error;
                    
                    % 결과 출력
                    fprintf('태그 %d 위치 추정 완료 - 시간: %.2f초, 오차: %.2f m\n', obj.ID, t, error);
                    
                    % 측정 거리 재설정
                    obj.MeasuredDistances = zeros(5, 1);
                end
            end
        end
        
        function processDSTWRFirstRound(obj, txPos, requestData, anchorIdx, t, anchorPlatforms, anchorModels, anchorPositions)
            % DS-TWR 첫 번째 왕복 처리
            %
            % 입력:
            %   txPos             - 태그의 현재 위치
            %   requestData       - TWR 요청 데이터
            %   anchorIdx         - 현재 앵커 인덱스
            %   t                 - 현재 시간
            %   anchorPlatforms   - 앵커 플랫폼 배열
            %   anchorModels      - 앵커 센서 모델 배열
            %   anchorPositions   - 앵커 위치 배열
            
            try
                % 전파 지연 계산
                distance = norm(txPos - anchorPositions(anchorIdx,:));
                propagationDelay = distance / physconst('LightSpeed');
                
                % 앵커에서의 요청 도착 시간 (T2)
                requestArrivalTime = t + propagationDelay;
                
                % 앵커의 처리 지연
                anchorProcessingDelay = 0.001; % 가정된 처리 지연
                if isfield(anchorModels{anchorIdx}, 'ProcessingDelay')
                    anchorProcessingDelay = anchorModels{anchorIdx}.ProcessingDelay;
                end
                responseTime = requestArrivalTime + anchorProcessingDelay;
                
                % 앵커 응답 생성
                [isSent, responseData] = anchorModels{anchorIdx}.sendTWRResponse(anchorPlatforms(anchorIdx), requestData, responseTime);
                
                if ~isSent
                    if obj.ShowDebug
                        fprintf('태그 %d - 앵커 %d DS-TWR 첫 번째 왕복: 응답 전송 실패\n', obj.ID, anchorIdx);
                    end
                    return;
                end
                
                % 태그에서의 응답 도착 시간 (T4)
                responseArrivalTime = responseTime + propagationDelay;
                
                % 첫 번째 왕복 시간 기록
                for i = 1:length(obj.PendingDSTWR)
                    if obj.PendingDSTWR(i).AnchorID == anchorIdx && obj.PendingDSTWR(i).Stage == 1
                        obj.PendingDSTWR(i).Timestamp2 = responseArrivalTime;
                        break;
                    end
                end
                
                % 첫 번째 왕복 시간 저장
                newRound = struct('AnchorID', anchorIdx, 'T1', t, 'T2', requestArrivalTime, 'T3', responseTime, 'T4', responseArrivalTime);
                
                % FirstRoundTimes이 비어있는지 확인
                if isempty(fieldnames(obj.FirstRoundTimes)) || isempty(obj.FirstRoundTimes)
                    obj.FirstRoundTimes = newRound;
                else
                    % 이전에 저장된 기록이 있는지 확인
                    found = false;
                    for i = 1:length(obj.FirstRoundTimes)
                        if obj.FirstRoundTimes(i).AnchorID == anchorIdx
                            obj.FirstRoundTimes(i) = newRound;
                            found = true;
                            break;
                        end
                    end
                    if ~found
                        obj.FirstRoundTimes(end+1) = newRound;
                    end
                end
                
                if obj.ShowDebug
                    fprintf('태그 %d - 앵커 %d DS-TWR 첫 번째 왕복 완료\n', obj.ID, anchorIdx);
                end
                
            catch e
                fprintf('태그 %d DS-TWR 첫 번째 왕복 처리 오류: %s\n', obj.ID, e.message);
            end
        end
        
        function processDSTWRSecondRound(obj, txPos, requestData, anchorIdx, t, anchorPlatforms, anchorModels, anchorPositions, pendingIdx)
            % DS-TWR 두 번째 왕복 처리
            %
            % 입력:
            %   txPos             - 태그의 현재 위치
            %   requestData       - TWR 요청 데이터
            %   anchorIdx         - 현재 앵커 인덱스
            %   t                 - 현재 시간
            %   anchorPlatforms   - 앵커 플랫폼 배열
            %   anchorModels      - 앵커 센서 모델 배열
            %   anchorPositions   - 앵커 위치 배열
            %   pendingIdx        - 처리 중인 DS-TWR 요청의 인덱스
            
            try
                % 전파 지연 계산
                distance = norm(txPos - anchorPositions(anchorIdx,:));
                propagationDelay = distance / physconst('LightSpeed');
                
                % 앵커에서의 요청 도착 시간 (T6)
                requestArrivalTime = t + propagationDelay;
                
                % 앵커의 처리 지연
                anchorProcessingDelay = 0.001; % 가정된 처리 지연
                if isfield(anchorModels{anchorIdx}, 'ProcessingDelay')
                    anchorProcessingDelay = anchorModels{anchorIdx}.ProcessingDelay;
                end
                responseTime = requestArrivalTime + anchorProcessingDelay;
                
                % 앵커 응답 생성
                [isSent, responseData] = anchorModels{anchorIdx}.sendTWRResponse(anchorPlatforms(anchorIdx), requestData, responseTime);
                
                if ~isSent
                    if obj.ShowDebug
                        fprintf('태그 %d - 앵커 %d DS-TWR 두 번째 왕복: 응답 전송 실패\n', obj.ID, anchorIdx);
                    end
                    return;
                end
                
                % 태그에서의 응답 도착 시간 (T8)
                responseArrivalTime = responseTime + propagationDelay;
                
                % 첫 번째 왕복 시간 찾기
                firstRoundIdx = -1;
                for i = 1:length(obj.FirstRoundTimes)
                    if obj.FirstRoundTimes(i).AnchorID == anchorIdx
                        firstRoundIdx = i;
                        break;
                    end
                end
                
                if firstRoundIdx > 0
                    % 첫 번째 왕복의 시간 정보
                    T1 = obj.FirstRoundTimes(firstRoundIdx).T1;
                    T2 = obj.FirstRoundTimes(firstRoundIdx).T2;
                    T3 = obj.FirstRoundTimes(firstRoundIdx).T3;
                    T4 = obj.FirstRoundTimes(firstRoundIdx).T4;
                    
                    % 두 번째 왕복의 시간 정보
                    T5 = t;
                    T6 = requestArrivalTime;
                    T7 = responseTime;
                    T8 = responseArrivalTime;
                    
                    % DS-TWR 거리 계산식
                    % 왕복 시간 계산
                    roundTrip1 = T4 - T1; % 첫 번째 왕복
                    roundTrip2 = T8 - T5; % 두 번째 왕복
                    
                    % 응답 시간 계산
                    replyTime1 = T3 - T2; % 첫 번째 응답 처리 시간
                    replyTime2 = T7 - T6; % 두 번째 응답 처리 시간
                    
                    % DS-TWR 공식으로 왕복 시간 계산 (크리스탈 오차 상쇄)
                    roundTripTime = (roundTrip1 * roundTrip2 - replyTime1 * replyTime2) / (roundTrip1 + roundTrip2 + replyTime1 + replyTime2);
                    
                    % 거리 계산
                    distance = roundTripTime * physconst('LightSpeed');
                    
                    % 측정된 거리 저장
                    obj.MeasuredDistances(anchorIdx) = distance;
                    
                    fprintf('태그 %d - 앵커 %d DS-TWR 두 번째 왕복 완료, 거리: %.2f m (실제: %.2f m)\n', ...
                            obj.ID, anchorIdx, distance, norm(txPos - anchorPositions(anchorIdx,:)));
                    
                    % 처리 완료된 DS-TWR 요청 제거
                    if pendingIdx <= length(obj.PendingDSTWR)
                        obj.PendingDSTWR(pendingIdx) = [];
                    end
                    
                    % 마지막 처리 시간 업데이트
                    obj.LastProcessedAnchors(anchorIdx) = t;
                else
                    if obj.ShowDebug
                        fprintf('태그 %d - 앵커 %d DS-TWR 첫 번째 왕복 정보를 찾을 수 없음\n', obj.ID, anchorIdx);
                    end
                end
                
            catch e
                fprintf('태그 %d DS-TWR 두 번째 왕복 처리 오류: %s\n', obj.ID, e.message);
            end
        end
        
        function filtered_pos = applySmoothing(obj, raw_position)
            % 단순 평활화 필터 적용 (노이즈 감소만을 위한 필터)
            %
            % 입력:
            %   raw_position - TWR로 추정된 원시 위치 [x, y, z]
            %
            % 출력:
            %   filtered_pos - 필터링된 위치 [x, y, z]
            
            % 벡터로 변환
            raw_position = raw_position(:);
            
            % 지수 평활화 필터 적용: 새 상태 = 알파 * 측정값 + (1-알파) * 이전 상태
            obj.FilterState = obj.FilterAlpha * raw_position + (1 - obj.FilterAlpha) * obj.FilterState;
            
            % 결과 반환
            filtered_pos = obj.FilterState;
        end
        
        function position = estimatePositionTWR(obj, anchorPositions, distances)
            % TWR 위치 추정 (Multilateration)
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
                warning('태그 %d: DS-TWR 위치 추정 데이터가 없습니다. 그래프를 그릴 수 없습니다.', obj.ID);
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
            figure('Name', sprintf('태그 %d - DS-TWR 위치 추적 결과', obj.ID));
            
            % X, Y, Z 좌표 비교
            subplot(2,2,1);
            plot(obj.TWRData.EstimatedTime, interpolatedTagPositions(:,1), 'g-', ...
                 obj.TWRData.EstimatedTime, obj.TWRData.EstimatedPosition(:,1), 'r--');
            title(sprintf('태그 %d - 북쪽(N) 좌표 비교', obj.ID));
            xlabel('시간 (초)'); ylabel('N 좌표 (m)');
            legend('실제 위치', 'DS-TWR 추정 위치');
            grid on;
            
            subplot(2,2,2);
            plot(obj.TWRData.EstimatedTime, interpolatedTagPositions(:,2), 'g-', ...
                 obj.TWRData.EstimatedTime, obj.TWRData.EstimatedPosition(:,2), 'r--');
            title(sprintf('태그 %d - 동쪽(E) 좌표 비교', obj.ID));
            xlabel('시간 (초)'); ylabel('E 좌표 (m)');
            legend('실제 위치', 'DS-TWR 추정 위치');
            grid on;
            
            subplot(2,2,3);
            plot(obj.TWRData.EstimatedTime, -interpolatedTagPositions(:,3), 'g-', ...
                 obj.TWRData.EstimatedTime, -obj.TWRData.EstimatedPosition(:,3), 'r--');
            title(sprintf('태그 %d - 높이(Height) 좌표 비교', obj.ID));
            xlabel('시간 (초)'); ylabel('높이 (m)');
            legend('실제 위치', 'DS-TWR 추정 위치');
            grid on;
            
            subplot(2,2,4);
            % 위치 오차 계산
            errors = sqrt(sum((interpolatedTagPositions - obj.TWRData.EstimatedPosition).^2, 2));
            plot(obj.TWRData.EstimatedTime, errors, 'b-');
            title(sprintf('태그 %d - DS-TWR 위치 추정 오차', obj.ID));
            xlabel('시간 (초)'); ylabel('오차 (m)');
            grid on;
            
            % 3D 경로 비교 (별도 그림)
            figure('Name', sprintf('태그 %d - DS-TWR 3D 경로 비교', obj.ID));
            plot3(interpolatedTagPositions(:,2), interpolatedTagPositions(:,1), -interpolatedTagPositions(:,3), ...
                  'g-', 'LineWidth', 2, 'DisplayName', '실제 경로');
            hold on;
            plot3(obj.TWRData.EstimatedPosition(:,2), obj.TWRData.EstimatedPosition(:,1), -obj.TWRData.EstimatedPosition(:,3), ...
                  'r--', 'LineWidth', 2, 'DisplayName', 'DS-TWR 추정 경로');
            plot3(anchorPositions(:,2), anchorPositions(:,1), -anchorPositions(:,3), ...
                  'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b', 'DisplayName', 'UWB 앵커');
            
            % 축 범위 설정
            xlim([0 15]);
            ylim([0 15]);
            zlim([0 5]);
            
            % 그래프 설정
            grid on;
            xlabel('동쪽 (m)'); ylabel('북쪽 (m)'); zlabel('높이 (m)');
            title(sprintf('태그 %d - DS-TWR 기반 UAV 위치추적 결과', obj.ID));
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
        
        function plotCombinedResults(obj, otherTags, anchorPositions)
            % 여러 태그의 결과를 하나의 그래프에 표시
            %
            % 입력:
            %   otherTags       - 다른 태그 객체 배열
            %   anchorPositions - 앵커 위치 행렬
            
            figure('Name', '모든 태그 DS-TWR 3D 경로 비교');
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
                      'DisplayName', sprintf('태그 %d DS-TWR 추정 경로', obj.ID));
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
                          'DisplayName', sprintf('태그 %d DS-TWR 추정 경로', tag.ID));
                end
            end
            
            % 앵커 위치 표시
            plot3(anchorPositions(:,2), anchorPositions(:,1), -anchorPositions(:,3), ...
                  'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k', 'DisplayName', 'UWB 앵커');
            
            % 그래프 설정
            grid on;
            xlabel('동쪽 (m)'); ylabel('북쪽 (m)'); zlabel('높이 (m)');
            title('DS-TWR 기반 다중 UAV 위치추적 결과');
            legend('show', 'Location', 'best');
            axis equal;
            xlim([0 15]);
            ylim([0 15]);
            zlim([0 5]);
            view(45, 30);
        end
    end
end