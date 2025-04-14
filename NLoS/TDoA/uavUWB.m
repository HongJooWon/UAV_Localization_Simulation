classdef uavUWB < uav.SensorAdaptor
    %uavUWB Adaptor that specifies behavior of UWB sensor in uavScenario
    %   This adaptor implements UWB sensor functionality for IEEE 802.15.4z
    %   standard in UAV scenarios. It can function as both a tag (transmitter)
    %   and an anchor (receiver) for TDOA-based localization.
    
    properties
        %UpdateRate Sensor Update Rate (Hz)
        UpdateRate = 100
        
        %PHYConfig HRP UWB waveform configuration
        PHYConfig
        
        %MACConfig IEEE 802.15.4 MAC configuration
        MACConfig
        
        %Blink Configured UWB blink signal
        Blink
        
        %Preamble Extracted preamble signal
        Preamble
        
        %FieldIndices Indices of different parts of the UWB signal
        FieldIndices
        
        %LastBlinkTime Time of the last blink transmission
        LastBlinkTime = 0
        
        %BlinkInterval Time between blinks (seconds)
        BlinkInterval = 0.1
        
        %IsSynchronized Flag indicating if time synchronization is available
        IsSynchronized = true
        
        %SynchronizationError Standard deviation of time sync error (seconds)
        SynchronizationError = 1e-9 % 1 nanosecond by default
        
        %SensorMode Operating mode of the sensor
        % 'tx': Transmit only, 'rx': Receive only, 'txrx': Both transmit and receive
        SensorMode = 'txrx'
        
        %DeviceID Unique identifier for this UWB device
        DeviceID = 1
        
        %ReceivedBlinks History of received blinks
        ReceivedBlinks = struct('TransmitterID', {}, 'BlinkID', {}, ...
            'TransmitTime', {}, 'ArrivalTime', {}, 'RSSI', {}, ...
            'TransmitterPosition', {})
        
        %MaxHistoryLength Maximum number of blinks to keep in history
        MaxHistoryLength = 100
        
        %DetectionThreshold RSSI threshold for valid detection (dBm)
        DetectionThreshold = -80

        % TWR 관련 속성
        TWRMode = false              % TWR 모드 활성화 플래그
        LastRequestTime = 0          % 마지막 요청 시간
        LastResponseTime = 0         % 마지막 응답 시간
        RequestInterval = 0.1        % 요청 간격 (초)
        ProcessingDelay = 10e-9      % 10 나노초 처리 지연
        PendingRequests = struct('Position', {}, 'Timestamp', {}, 'MessageType', {}, ...
                               'TransmitterID', {}, 'TargetID', {}, 'Waveform', {}, ...
                               'Preamble', {}, 'PHYConfig', {})   % 대기 중인 요청 관리 구조체 초기화
        MeasuredDistances = struct() % 측정된 거리 저장 - 구조체로 변경
        TargetAnchors = []           % 측정 대상 앵커 ID 목록
    end
    
    methods
        function obj = uavUWB(deviceID, mode, twrMode)
            %uavUWB Creates a UWB sensor adaptor
            %   UWB = uavUWB() creates a UWB sensor with default settings
            %   UWB = uavUWB(DEVICEID) creates a UWB sensor with specified ID
            %   UWB = uavUWB(DEVICEID, MODE) also specifies operating mode
            %   ('tx', 'rx', or 'txrx')
            
            % Call superclass constructor with empty sensor model
            obj@uav.SensorAdaptor([]);
            
            % Set device ID if provided
            if nargin > 0 && ~isempty(deviceID)
                obj.DeviceID = deviceID;
            end
            
            % Set operation mode if provided
            if nargin > 1 && ~isempty(mode)
                if ismember(mode, {'tx', 'rx', 'txrx'})
                    obj.SensorMode = mode;
                else
                    warning('Invalid mode specified. Using default ''txrx''.');
                end
            end
            
            % Configure UWB parameters if needed for transmission
            if ismember(obj.SensorMode, {'tx', 'txrx'})
                obj.configureUWB();
            end

            % TWR 모드 설정 옵션 추가
            if nargin > 2 && ~isempty(twrMode)
                obj.TWRMode = twrMode;
                if obj.TWRMode && strcmp(obj.SensorMode, 'rx')
                    warning('TWR 모드에서는 송수신 모드가 필요합니다. SensorMode를 txrx로 변경합니다.');
                    obj.SensorMode = 'txrx';
                end
            end
        end
        
        function configureUWB(obj)
            %configureUWB Configure UWB signal parameters
            
            % MAC layer configuration for a simple blink
            payload = '00'; % Simple payload
            obj.MACConfig = lrwpan.MACFrameConfig( ...
                FrameType='Data', ...
                SourceAddressing='Short address', ...
                SourcePANIdentifier='AB12', ...
                SourceAddress='CD77');
            
            % Generate MAC frame
            macFrame = lrwpan.MACFrameGenerator(obj.MACConfig, payload);
            
            % PHY layer configuration (IEEE 802.15.4z HRP UWB)
            obj.PHYConfig = lrwpanHRPConfig('Mode', 'HPRF', ...
                                         'STSPacketConfiguration', 1, ...
                                         'PSDULength', length(macFrame)/8, ...
                                         'Ranging', true);
            
            % Generate UWB waveform (blink)
            obj.Blink = lrwpanWaveformGenerator(macFrame, obj.PHYConfig);
            
            % Calculate field indices
            obj.FieldIndices = lrwpanHRPFieldIndices(obj.PHYConfig);
            
            % Extract preamble (1st instance of Nsync repetitions)
            obj.Preamble = obj.Blink(1:obj.FieldIndices.SYNC(end)/obj.PHYConfig.PreambleDuration);
        end
        
        function setup(obj, scene, platform, sensor)
            %setup Initialize the sensor in the scenario
            
            % Match sensor update rate to scenario update rate
            obj.UpdateRate = scene.UpdateRate;
        end
        
        function [isUpdated, t, sensorData] = read(obj, scene, platform, sensor, t)
            % 간소화된 read 메서드
            % 모든 자세한 처리 로직은 그대로 유지하되, 출력 형식만 단순화
            
            % 송신 모드 처리
            if ismember(obj.SensorMode, {'tx', 'txrx'})
                [isTx, txData] = obj.processTransmission(platform, t);
            else
                isTx = false;
                txData = [];
            end
            
            % 수신 모드 처리
            if ismember(obj.SensorMode, {'rx', 'txrx'})
                [isRx, rxData] = obj.processReception(scene, platform, t);
            else
                isRx = false;
                rxData = [];
            end
            
            % 센서가 업데이트 되었는지 여부
            isUpdated = isTx || isRx;
            
            % 단순화된 센서 데이터: 
            % MATLAB uavSensor가 단일 값만 처리할 수 있다면 중요한 상태만 반환
            if isRx
                sensorData = 1;  % 수신 발생
            elseif isTx
                sensorData = 2;  % 송신 발생
            else
                sensorData = 0;  % 아무 일도 없음
            end
        end

        function [isNLoS, additionalDelay, additionalAttenuation] = checkNLoS(obj, transmitterPos, receiverPos, obstacles)
            % Check if there's a non-line-of-sight condition between transmitter and receiver
            % 
            % Inputs:
            %   transmitterPos - 3D position of the transmitter [x, y, z]
            %   receiverPos   - 3D position of the receiver [x, y, z]
            %   obstacles     - Array of obstacle structures with fields:
            %                   - position: [x, y, z] center position
            %                   - dimensions: [width_x, width_y, height] dimensions
            % 
            % Outputs:
            %   isNLoS        - Boolean indicating if there's an obstacle in the path
            %   additionalDelay - Additional signal delay due to NLoS (seconds)
            %   additionalAttenuation - Additional signal attenuation due to NLoS (dB)
            
            % Default return values
            isNLoS = false;
            additionalDelay = 0;
            additionalAttenuation = 0;
            
            % Direction vector from transmitter to receiver
            direction = receiverPos - transmitterPos;
            distance = norm(direction);
            direction = direction / distance; % Normalize to unit vector
            
            % Check intersection with each obstacle
            for i = 1:length(obstacles)
                obstacle = obstacles(i);
                
                % Get obstacle dimensions and position
                pos = obstacle.position;
                dim = obstacle.dimensions;
                
                % Calculate obstacle boundaries
                minBound = pos - dim/2;
                maxBound = pos + dim/2;
                
                % Ray-box intersection algorithm
                t_min = -Inf;
                t_max = Inf;
                
                % Check intersection along each axis
                for axis = 1:3
                    if abs(direction(axis)) < 1e-6
                        % Ray is parallel to this axis-aligned plane
                        if transmitterPos(axis) < minBound(axis) || transmitterPos(axis) > maxBound(axis)
                            % Ray origin is outside the slab, so no intersection
                            continue;
                        end
                    else
                        % Compute intersection with the two planes perpendicular to the current axis
                        t1 = (minBound(axis) - transmitterPos(axis)) / direction(axis);
                        t2 = (maxBound(axis) - transmitterPos(axis)) / direction(axis);
                        
                        % Ensure t1 <= t2
                        if t1 > t2
                            temp = t1;
                            t1 = t2;
                            t2 = temp;
                        end
                        
                        % Update interval
                        t_min = max(t_min, t1);
                        t_max = min(t_max, t2);
                        
                        if t_min > t_max
                            % No intersection with this obstacle
                            continue;
                        end
                    end
                end
                
                % Check if intersection point is along the segment between transmitter and receiver
                if t_min <= distance && t_max >= 0
                    isNLoS = true;
                    
                    % Calculate additional attenuation based on material (concrete-like)
                    % Typically 10-30 dB for a concrete wall at UWB frequencies
                    materialAttenuation = 15; % dB
                    additionalAttenuation = materialAttenuation;
                    
                    % Calculate additional delay
                    % Signal travels slower through materials (concrete has ~n=2.5)
                    materialIndex = 2.5; 
                    penetrationDistance = min(dim); % Use minimum dimension as wall thickness
                    timeInMaterial = penetrationDistance * materialIndex / physconst('LightSpeed');
                    timeInAir = penetrationDistance / physconst('LightSpeed');
                    additionalDelay = timeInMaterial - timeInAir;
                    
                    break; % Stop checking after finding first obstacle
                end
            end
        end
        
        function [isTransmitting, blinkData] = processTransmission(obj, platform, t)
            %processTransmission Handle UWB signal transmission
            
            % 플랫폼 객체에서 위치 가져오기
            try
                if isprop(platform, 'Position') && ~isempty(platform.Position)
                    position = platform.Position;
                elseif isprop(platform, 'Trajectory') && ~isempty(platform.Trajectory)
                    [position, ~, ~, ~, ~] = lookupPose(platform.Trajectory, t);
                else
                    [motion, ~] = read(platform);
                    position = motion(1:3);
                end
            catch e
                fprintf('  위치 가져오기 오류: %s\n', e.message);
                position = [0, 0, 0];
            end
            
            % 기본 블링크 데이터 구조
            blinkData = struct('Position', position, ...
                              'Timestamp', t, ...
                              'BlinkID', round(t / obj.BlinkInterval), ... % 항상 유효한 ID 설정
                              'TransmitterID', obj.DeviceID, ...
                              'Waveform', obj.Blink, ...
                              'Preamble', obj.Preamble, ...
                              'PHYConfig', obj.PHYConfig);
            
            % 블링크 전송 조건 확인 - 수정된 부분: 항상 확인하도록 디버깅
            timeSinceLastBlink = t - obj.LastBlinkTime;
            % fprintf('  블링크 조건 확인: 경과 시간=%.6f, 설정 간격=%.6f\n', timeSinceLastBlink, obj.BlinkInterval);
            
            % 일반적인 블링크 간격 기반 조건
            if timeSinceLastBlink >= obj.BlinkInterval - 0.0001
                isTransmitting = true;
                obj.LastBlinkTime = t; % 마지막 블링크 시간 업데이트
                % fprintf('  블링크 전송 결정: isTransmitting=true\n');
            else
                % 블링크 간격 충족되지 않음 - 전송하지 않음
                isTransmitting = false;
                % fprintf('  블링크 간격 미충족 - 전송하지 않음\n');
            end
        end
        
        function [isReceiving, receptionData] = processReception(obj, scene, platform, t)
            %processReception Handle UWB signal reception
            
            % Default - not receiving
            isReceiving = false;
            receptionData = struct('TransmitterID', [], 'BlinkID', [], ...
                'TransmitTime', [], 'ArrivalTime', [], 'RSSI', [], ...
                'TransmitterPosition', []);
            
            % Get all platforms in the scenario
            platforms = scene.Platforms;
            
            % Check each platform for active UWB transmissions
            for i = 1:length(platforms)
                % Skip the current platform (our own platform)
                if platforms(i) == platform
                    continue;
                end
                
                % Get all sensors on this platform
                allSensors = platforms(i).Sensors;
                
                % Find UWB sensors that might be transmitting
                for j = 1:length(allSensors)
                    % Get the sensor model
                    try
                        sensorModel = allSensors(j).SensorModel;
                        
                        % Check if it's a uavUWB sensor in tx or txrx mode
                        if ~isa(sensorModel, 'uavUWB') || ...
                                strcmp(sensorModel.SensorMode, 'rx')
                            continue;
                        end
                        
                        % Read the sensor to check for transmissions
                        try
                            % Different handling based on the transmitter's mode
                            if strcmp(sensorModel.SensorMode, 'tx')
                                [isTx, ~, sensorData] = allSensors(j).read();
                                txData = sensorData.TxData;
                            else % 'txrx'
                                [isTx, ~, sensorData] = allSensors(j).read();
                                txData = sensorData.TxData;
                            end
                            
                            % If transmitting a blink
                            if isTx && sensorData.TxStatus
                                % Process the received signal
                                % Process the received signal
                                receivedSignal = obj.simulateReception(platforms(i).Position, ...
                                    txData, platform.Position, t);
                                
                                % If signal was detected
                                if receivedSignal.RSSI > obj.DetectionThreshold
                                    isReceiving = true;
                                    receptionData = receivedSignal;
                                    
                                    % Add to reception history
                                    obj.addToHistory(receptionData);
                                    
                                    % Only process one reception per read call
                                    % (in real systems, collision would occur)
                                    return;
                                end
                            end
                        catch e
                            % Log error and continue
                            warning('Error reading sensor: %s', e.message);
                        end
                    catch
                        % Not a sensor with a SensorModel, skip
                        continue;
                    end
                end
            end
        end
        
        function receivedSignal = simulateReception(obj, transmitterPos, txData, receiverPos, currentTime, obstacles)
            % simulateReception Simulate signal propagation and reception with NLoS effects
            %
            % Inputs:
            %   transmitterPos - Position of the transmitter
            %   txData        - Transmitted signal data
            %   receiverPos   - Position of the receiver
            %   currentTime   - Current simulation time
            %   obstacles     - Array of obstacle structures (optional)
            
            % Calculate distance
            distance = norm(transmitterPos - receiverPos);
            
            % Initialize NLoS variables
            isNLoS = false;
            additionalDelay = 0;
            additionalAttenuation = 0;
            
            % Check for NLoS conditions if obstacles are provided
            if nargin >= 6 && ~isempty(obstacles)
                [isNLoS, additionalDelay, additionalAttenuation] = obj.checkNLoS(transmitterPos, receiverPos, obstacles);
            end
            
            % Calculate propagation delay (include additional delay if NLoS)
            propDelay = distance / physconst('LightSpeed') + additionalDelay;
            
            % Add synchronization error if synchronized
            if obj.IsSynchronized
                timeError = obj.SynchronizationError * randn();
            else
                timeError = 0; % Unsynchronized systems measure relative time
            end
            
            % Create result structure
            receivedSignal = struct('TransmitterID', txData.TransmitterID, ...
                                  'BlinkID', txData.BlinkID, ...
                                  'TransmitTime', txData.Timestamp, ...
                                  'ArrivalTime', txData.Timestamp + propDelay + timeError, ...
                                  'TransmitterPosition', transmitterPos, ...
                                  'IsNLoS', isNLoS);
            
            % Calculate received signal strength (simple path loss model)
            % Free space path loss: FSPL(dB) = 20*log10(d) + 20*log10(f) - 147.55
            % where d is distance in meters and f is frequency in Hz
            frequency = 6.5e9; % UWB center frequency (6.5 GHz)
            txPower = -41.3;   % UWB transmission power (dBm)
            
            % Calculate path loss with some randomness for fading
            pathLoss = 20*log10(distance) + 20*log10(frequency) - 147.55;
            fadingdB = 3 * randn(); % Random fading effect
            
            % Apply additional attenuation if NLoS
            totalLoss = pathLoss + additionalAttenuation;
            
            % Calculate RSSI (with gain boost for simulation purposes)
            gainBoost = 30;
            receivedSignal.RSSI = txPower - totalLoss + fadingdB + gainBoost;
        end
        
        function tdoa = calculateTDOA(obj, referenceDeviceID)
            %calculateTDOA Calculate TDOA values relative to a reference device
            %   TDOA = calculateTDOA(OBJ, REFERENCEDEVICEID) calculates
            %   the time difference of arrival between blinks from
            %   different devices, using the specified reference device.
            
            % Initialize empty TDOA result
            tdoa = struct('DeviceID', {}, 'BlinkID', {}, 'TDOA', {});
            
            % Check if we have enough receptions
            if length(obj.ReceivedBlinks) < 2
                warning('Not enough receptions to calculate TDOA.');
                return;
            end
            
            % Find all unique transmitter IDs in our recent receptions
            allIDs = [obj.ReceivedBlinks.TransmitterID];
            uniqueIDs = unique(allIDs);
            
            % Check if we have the reference device
            if ~ismember(referenceDeviceID, uniqueIDs)
                warning('Reference device ID not found in recent receptions.');
                return;
            end
            
            % Get the most recent reception from the reference device
            refIdx = find([obj.ReceivedBlinks.TransmitterID] == referenceDeviceID, 1, 'first');
            if isempty(refIdx)
                warning('No recent receptions from reference device.');
                return;
            end
            refReception = obj.ReceivedBlinks(refIdx);
            
            % Calculate TDOA for all other devices
            for i = 1:length(uniqueIDs)
                deviceID = uniqueIDs(i);
                
                % Skip the reference device
                if deviceID == referenceDeviceID
                    continue;
                end
                
                % Find the most recent reception from this device
                devIdx = find([obj.ReceivedBlinks.TransmitterID] == deviceID, 1, 'first');
                if isempty(devIdx)
                    continue;
                end
                devReception = obj.ReceivedBlinks(devIdx);
                
                % Calculate TDOA
                tdoaValue = devReception.ArrivalTime - refReception.ArrivalTime;
                
                % Add to results
                newTDOA = struct('DeviceID', deviceID, ...
                    'BlinkID', devReception.BlinkID, ...
                    'TDOA', tdoaValue);
                tdoa = [tdoa, newTDOA];
            end
        end

         % TWR 요청 전송 메서드 추가
        function [isSent, requestData] = sendTWRRequest(obj, platform, targetID, t)
            isSent = false;
            requestData = struct();
            
            % 요청 전송 간격 확인
            if t - obj.LastRequestTime < obj.RequestInterval
                return;
            end
            
            % 플랫폼 위치 가져오기
            try
                if isprop(platform, 'Position') && ~isempty(platform.Position)
                    position = platform.Position;
                elseif isprop(platform, 'Trajectory') && ~isempty(platform.Trajectory)
                    [position, ~, ~, ~, ~] = lookupPose(platform.Trajectory, t);
                else
                    [motion, ~] = read(platform);
                    position = motion(1:3);
                end
            catch
                position = [0, 0, 0];
            end
            
            % TWR 요청 데이터 구성
            requestData = struct('Position', position, ...
                                'Timestamp', t, ...
                                'MessageType', 'TWR_REQUEST', ...
                                'TransmitterID', obj.DeviceID, ...
                                'TargetID', targetID, ...
                                'Waveform', obj.Blink, ...
                                'Preamble', obj.Preamble, ...
                                'PHYConfig', obj.PHYConfig);
            
            % 요청 전송 기록
            obj.LastRequestTime = t;
            
            % 보낸 요청 추적 - 수정된 부분
            if isempty(fieldnames(obj.PendingRequests)) || numel(obj.PendingRequests) == 0
                obj.PendingRequests = requestData;
            else
                % 구조체 배열 확장
                requiredFields = fieldnames(requestData);
                existingFields = fieldnames(obj.PendingRequests);
                
                % 필드 검증
                if isequal(sort(requiredFields), sort(existingFields))
                    % 필드가 일치하면 배열에 추가
                    obj.PendingRequests(end+1) = requestData;
                else
                    % 필드가 일치하지 않으면 기존 구조체 재구성
                    tempRequests = obj.PendingRequests;
                    obj.PendingRequests = struct();
                    for i = 1:numel(tempRequests)
                        newStruct = struct();
                        for j = 1:numel(requiredFields)
                            field = requiredFields{j};
                            if isfield(tempRequests, field)
                                newStruct.(field) = tempRequests(i).(field);
                            else
                                newStruct.(field) = [];
                            end
                        end
                        if i == 1
                            obj.PendingRequests = newStruct;
                        else
                            obj.PendingRequests(i) = newStruct;
                        end
                    end
                    obj.PendingRequests(end+1) = requestData;
                end
            end
            
            isSent = true;
        end

        % TWR 응답 전송 메서드 추가
        function [isSent, responseData] = sendTWRResponse(obj, platform, requestData, t)
            isSent = false;
            responseData = struct();
            
            % 요청이 자신에게 온 것인지 확인
            if requestData.TargetID ~= obj.DeviceID
                return;
            end
            
            % 플랫폼 위치 가져오기
            try
                if isprop(platform, 'Position') && ~isempty(platform.Position)
                    position = platform.Position;
                elseif isprop(platform, 'Trajectory') && ~isempty(platform.Trajectory)
                    [position, ~, ~, ~, ~] = lookupPose(platform.Trajectory, t);
                else
                    [motion, ~] = read(platform);
                    position = motion(1:3);
                end
            catch
                position = [0, 0, 0];
            end
            
            % TWR 응답 데이터 구성
            responseData = struct('Position', position, ...
                                 'Timestamp', t, ...
                                 'MessageType', 'TWR_RESPONSE', ...
                                 'TransmitterID', obj.DeviceID, ...
                                 'TargetID', requestData.TransmitterID, ...
                                 'RequestTimestamp', requestData.Timestamp, ...
                                 'ProcessingDelay', obj.ProcessingDelay, ...
                                 'Waveform', obj.Blink, ...
                                 'Preamble', obj.Preamble, ...
                                 'PHYConfig', obj.PHYConfig);
            
            % 응답 전송 기록
            obj.LastResponseTime = t;
            
            isSent = true;
        end
        
        % TWR 응답 처리 메서드 추가
        function processTWRResponse(obj, responseData, t)
            % 요청 찾기
            requestIdx = 0;
            for i = 1:length(obj.PendingRequests)
                if obj.PendingRequests(i).TargetID == responseData.TransmitterID
                    requestIdx = i;
                    break;
                end
            end
            
            if requestIdx == 0
                return; % 요청을 찾을 수 없음
            end
            
            % 왕복 시간 계산
            roundTripTime = t - obj.PendingRequests(requestIdx).Timestamp;
            
            % 처리 지연 시간 제외
            adjustedTime = roundTripTime - responseData.ProcessingDelay;
            
            % 거리 계산 (단방향 시간 = 왕복 시간 / 2)
            distance = (adjustedTime / 2) * physconst('LightSpeed');
            
            % 거리 저장 - 필드 이름 수정 ('anchor_' 접두사 추가)
            fieldName = ['anchor_', num2str(responseData.TransmitterID)];
            obj.MeasuredDistances.(fieldName) = distance;
            
            % 처리된 요청 제거
            if length(obj.PendingRequests) == 1
                obj.PendingRequests = struct('Position', {}, 'Timestamp', {}, 'MessageType', {}, ...
                                          'TransmitterID', {}, 'TargetID', {}, 'Waveform', {}, ...
                                          'Preamble', {}, 'PHYConfig', {});
            else
                obj.PendingRequests(requestIdx) = [];
            end
        end
        
        function out = getEmptyOutputs(obj)
            %getEmptyOutputs Define empty outputs
            
            % Create empty transmission data
            emptyTxData = struct('Position', [0 0 0], ...
                'Timestamp', 0, ...
                'BlinkID', 0, ...
                'TransmitterID', obj.DeviceID, ...
                'Waveform', [], ...
                'Preamble', [], ...
                'PHYConfig', []);
            
            % Create empty reception data
            emptyRxData = struct('TransmitterID', [], ...
                'BlinkID', [], ...
                'TransmitTime', [], ...
                'ArrivalTime', [], ...
                'RSSI', [], ...
                'TransmitterPosition', []);
            
            % Create consistent empty output structure
            sensorData = struct();
            
            % Fill based on mode
            switch obj.SensorMode
                case 'tx'
                    sensorData.TxStatus = false;
                    sensorData.TxData = emptyTxData;
                case 'rx'
                    sensorData.RxStatus = false;
                    sensorData.RxData = emptyRxData;
                case 'txrx'
                    sensorData.TxStatus = false;
                    sensorData.TxData = emptyTxData;
                    sensorData.RxStatus = false;
                    sensorData.RxData = emptyRxData;
            end
            
            % Return consistent output format
            out = {false, 0, sensorData};
        end
        
        function reset(obj)
            %reset Reset the sensor state
            obj.LastBlinkTime = 0;
            obj.ReceivedBlinks = struct('TransmitterID', {}, 'BlinkID', {}, ...
                'TransmitTime', {}, 'ArrivalTime', {}, 'RSSI', {}, ...
                'TransmitterPosition', {});
        end
        
        function waveform = getBlinkWaveform(obj)
            %getBlinkWaveform Return the current blink waveform
            waveform = obj.Blink;
        end
        
        function preamble = getBlinkPreamble(obj)
            %getBlinkPreamble Return the blink preamble
            preamble = obj.Preamble;
        end
        
        function config = getPhyConfig(obj)
            %getPhyConfig Return the PHY configuration
            config = obj.PHYConfig;
        end
        
        function history = getReceptionHistory(obj)
            %getReceptionHistory Return history of received blinks
            history = obj.ReceivedBlinks;
        end
    end
end