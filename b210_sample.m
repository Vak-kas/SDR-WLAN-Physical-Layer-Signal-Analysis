if exist('rx','var')
    release(rx);
    clear rx;
end
clear;
clc;

%% 기기 연결 확인
radios = findsdru();


%% 초기 변수 설정
%=====채널=====
frequencyBand = 5;
channelNumber = 36;

%=====실험 변수====
threshold = 0.7;  %탐지 임계값 설정
maxLoops = 20; %실험 반복 횟수
captureTime = milliseconds(100); % 1000 = 1초
justOne = false; % 동작 모드 변경

%====패킷 수신 관련====
sampleRate =  20; %샘플레이트 (채널 밴드)
decimationFactor = 1; %다운샘플링 비율

gain = 20; %RF신호를 얼마나 증폭해서 받을 것인가 (dB)
skip = 2000;








%% 초기 initial
radio = radios(1);
fc = wlanChannelFrequency(channelNumber, frequencyBand);
sampleRate = sampleRate * 1e6;
cbw = sampleRateToCBW(sampleRate);

captureTimeSec = seconds(captureTime);
samplePerFrame = round(sampleRate * captureTimeSec); %한 번에 몇 개 샘플을 가져오는가(묶음 크기)
maxBufferLen = round(sampleRate * 0.2); % 200ms

if strcmp(radio.Status, "Success")

        rx = comm.SDRuReceiver( ...
        Platform = radio.Platform, ... %장비명(b210)
        SerialNum = radio.SerialNum, ... %시리얼번호
        CenterFrequency = fc, ... %중앙 주파수
        MasterClockRate = sampleRate * decimationFactor, ... %내부 샘플 생성 속도
        DecimationFactor = decimationFactor, ... %디지털 다운샘플링
        Gain = gain, ... %아날로그 증폭
        SamplesPerFrame = samplePerFrame, ... %버퍼링
        OutputDataType = 'double' ... %매트랩전달형식
        );

else
    error('Device connection failed. Please check the hardware and try again.');
end


%% 그래프 + 실험 환경 설정

%그래프+실험 세팅
fig = figure(1);
maxHistory = 2; % 최근 패킷 몇 개 볼지
displayLimit = 8000; % 시각화할 신호의 최대 길이
stackView = true; %true시 상하 , false시 좌우 배치


plotHandles = gobjects(maxHistory,1);
specHandles = gobjects(maxHistory,1);
for k = 1:maxHistory
    if stackView
        % --- Time domain ---
        subplot(maxHistory*2, 1, (k-1)*2 + 1);
        plotHandles(k) = plot(NaN, NaN);
        grid on;
        ylabel('Amp');
        title(sprintf('Packet #%d - Time Domain', k));

        % --- Spectrogram ---
        subplot(maxHistory*2, 1, (k-1)*2 + 2);
        specHandles(k) = imagesc(NaN);
        axis xy;
        ylabel('Freq');
        title(sprintf('Packet #%d - Spectrogram', k));

    else
        % 기존 구조 유지
        subplot(maxHistory,2,(k-1)*2+1);
        plotHandles(k) = plot(NaN,NaN);
        grid on;

        subplot(maxHistory,2,(k-1)*2+2);
        specHandles(k) = imagesc(NaN);
        axis xy;
    end
end
packetHistory = cell(maxHistory,1);


% 데이터 수신 및 패킷 처리
packetCount = 0;
stopAll = false;
buffer = [];



%% 신호 수신 및 실시간 감지 루프
disp("Scanning for Wi-Fi signals...");
for i = 1:maxLoops
    
    % 데이터 수신
    if stopAll, break; end
    [data, len] = rx();
    if len <=0, continue; end
    
    % buffer에 누적
    buffer = [buffer; data];

    if length(buffer) > maxBufferLen
        buffer = buffer(end-maxBufferLen+1:end);
    end
    

    % 신호 수신 및 처리 루프
    disp("Receiving signals...");
    idx = wlanPacketDetect(buffer, cbw, 0, threshold); %coarse candidate.
    
    while ~isempty(idx) && all(idx > 0)
        
        %% 패킷 추출
        extractLen = 50000; 
        if idx(1) + extractLen > size(buffer, 1), break; end

        rxPacket = buffer(idx : idx + extractLen - 1);

        %% LLTF 기반 정밀 동기화 (Fine Timing) - 시간 동기화
        [cpLen, symLen] = extractLLTFLengths(sampleRate);
        cfg = wlanNonHTConfig(ChannelBandwidth=cbw);
        % lltf = wlanLLTF(cfg);
        % lltfRef = lltf(cpLen+1 : cpLen+symLen); %cp 이후

        fineOffset = wlanSymbolTimingEstimate(rxPacket, cbw);
        if fineOffset >= 1 && fineOffset < length(rxPacket)
            rxPacket = rxPacket(fineOffset:end);
        else
            % 타이밍 추정 실패 시: 현재 idx에서 조금만 전진해서 다시 찾기
            fprintf("Timing Estimate Failed. Advancing buffer...\n");
            [buffer, idx] = advanceBuffer(buffer, idx + skip, cbw, threshold, sampleRate);
            continue;
        end

        %% CFO 보정 (주파수 동기화)
        cfoCoarse = wlanCoarseCFOEstimate(rxPacket, cbw); %Coarse : STF기반
        rxPacket = frequencyOffset(rxPacket, sampleRate, -cfoCoarse);

        cfoFine = wlanFineCFOEstimate(rxPacket, cbw); %Fine : LTF 기반
        rxPacket = frequencyOffset(rxPacket, sampleRate, -cfoFine);
        
        % 전체 오차 합산
        totalCFO = cfoCoarse + cfoFine;
        fprintf("CFO Details -> Coarse: %.2f Hz| Fine: %.2f Hz | Total: %.2f Hz\n", ...
                cfoCoarse, cfoFine, totalCFO);
        
        %% 필드 인덱스 재설정 및 채널 추정 (CSI)
        ind = wlanFieldIndices(cfg); %L-STF/L-LTF/L-SIG/Data 시작 위치 인덱스
        if ind.LSIG(2) > length(rxPacket), continue; end
        
        % 채널 추정치 추출
        ltfField = rxPacket(ind.LLTF(1):ind.LLTF(2));
        demodLTF = wlanLLTFDemodulate(ltfField, cbw);
        chanEst = wlanLLTFChannelEstimate(demodLTF, cbw);


        % % 채널의 크기 응답 (어떤 주파수가 잘 통과했나?)
        % figure;
        % plot(abs(chanEst));
        % title("CSI Magnitude");
        % xlabel("Subcarrier");
        % ylabel("Magnitude");
        % 
        % % 채널의 위상 응답 (어떤 주파수가 얼마나 지연되었나?)
        % figure;
        % plot(angle(chanEst));
        % title("CSI Phase");
        % xlabel("Subcarrier");
        % ylabel("Phase");

        %% L-SIG 복원 및 포멧 판별
        try
            lsig = rxPacket(ind.LSIG(1):ind.LSIG(2));
            [lsigBits, fail] = wlanLSIGRecover(lsig, chanEst, 0.1, cbw);

            if fail
                fprintf("L-SIG Recover Failed. Advancing buffer...\n");
                [buffer, idx] = advanceBuffer(buffer, idx + skip, cbw, threshold, sampleRate);
                continue;
            end


            %% Format Detect용 구간 생성
            numExtraSymbols = 3;
            ofdmSymbolLen = round(4e-6 * sampleRate);  % Non-HT OFDM symbol = 4us
            formatEnd = ind.LSIG(2) + numExtraSymbols * ofdmSymbolLen;
            
            if formatEnd > length(rxPacket)
                fprintf("Format Detected Failed. Advancing buffer...\n");
                [buffer, idx] = advanceBuffer(buffer, idx + skip, cbw, threshold, sampleRate);
                continue;

            end
            
            fmtDetect = rxPacket(ind.LSIG(1) : formatEnd);
            format = wlanFormatDetect(fmtDetect, chanEst, 0.2, cbw);
            fprintf("Detected Format: %s\n", string(format));


            %% 포맷 판별 및 MAC 디코딩
            % 기본 전진 거리는 skip으로 설정 (실패 대비)
            shift = skip; 
            
            switch format
                case "Non-HT"
                    % L-SIG 기반 PSDU 길이 설정
                    psduLen = double(bit2int(lsigBits(6:17), 12, 0));
                    cfg.PSDULength = psduLen;
                    
                    % 데이터 필드 인덱스 계산
                    indData = wlanFieldIndices(cfg, 'NonHT-Data');
                    
                    % 버퍼에 데이터 끝까지 들어있는지 확인
                    if indData(2) <= length(rxPacket)
                        rxData = rxPacket(indData(1):indData(2));
                        
                        % 데이터 복조 (MPDU 비트 추출)
                        bits = wlanNonHTDataRecover(rxData, chanEst, 0.2, cfg);
                        
                        % MAC 계층 디코딩
                        [cfgMAC, ~, decodeStatus] = wlanMPDUDecode(bits, cfg, 'SuppressWarnings', true);
                        
                        if ~decodeStatus
                            % 성공 시 전진 거리를 패킷 끝으로 업데이트
                            shift = indData(2); 
                            
                            % 결과 출력
                            if strcmp(cfgMAC.FrameType, 'Beacon')
                                fprintf("<strong>[FOUND] SSID: %s | BSSID: %s</strong>\n", ...
                                    string(cfgMAC.ManagementConfig.SSID), string(cfgMAC.Address3));
                            else
                                % 비콘이 아닌 다른 프레임 타입 확인용
                                fprintf("Frame Type: %s (Detected)\n", string(cfgMAC.FrameType));
                            end
                        end
                    end
                    
                otherwise
                    % HT-Mixed, VHT 등은 일단 skip
                    fprintf("Detected Format: %s (Skipping...)\n", string(format));
                    shift = skip; 
            end
            
            % [통합 지점] advanceBuffer 하나로 버퍼 밀기 + 다음 패킷 찾기 완료
            [buffer, idx] = advanceBuffer(buffer, idx + shift, cbw, threshold, sampleRate);

        catch
            fprintf("Error occurred: %s. Advancing buffer...\n", ME.message);
            % 어떤 에러가 나도 idx + skip으로 밀어줘서 무한 루프 방지
            [buffer, idx] = advanceBuffer(buffer, idx + skip, cbw, threshold, sampleRate);
            continue;
        end
        %% AFTER 추가 예정
        %-------------
        packetCount = packetCount + 1;


        % ==== 패킷 저장 (히스토리 유지) ====
        
        packetHistory = [{rxPacket}; packetHistory(1:end-1)];
        
        % ==== 시각화 ====
        set(0,'CurrentFigure',fig);
        
        for k = 1:maxHistory
            if isempty(packetHistory{k})
                continue;
            end
        
            pkt = packetHistory{k};
            actualLimit = min(length(pkt), displayLimit);
            t = (0:actualLimit-1) / sampleRate * 1e6;
            sig = abs(pkt(1:actualLimit));
        
            % --- Time domain ---
            subplot(maxHistory*2, 1, (k-1)*2 + 1);
            set(plotHandles(k), 'XData', t, 'YData', sig);
            title(sprintf('Packet #%d (latest-%d)', packetCount-k+1, k-1));
            xlim([0 t(end)]);
            ylim([0 max(sig)*1.2 + 1e-6]);
        
            % --- Spectrogram ---
            subplot(maxHistory*2, 1, (k-1)*2 + 2);
            [S,F,T] = spectrogram(pkt(1:actualLimit),256,200,256,sampleRate,'centered');
            imagesc(T*1e6, F/1e6, 20*log10(abs(S)));
            axis xy;
            xlabel('Time (μs)');
            ylabel('Freq (MHz)');
        end
        
        drawnow limitrate;

        %  ================================
        if justOne, stopAll = true; break; end
        
        if idx + skip < size(buffer,1)
            buffer = buffer(idx + skip:end,:);
        else
            buffer = [];
        end
        
        idx = wlanPacketDetect(buffer, sampleRateToCBW(sampleRate), 0, threshold);
        
    end
end

disp("탐색 종료");

release(rx); % 하드웨어 해제 





%% function 
function cbw = sampleRateToCBW(sampleRate)

    % Hz 기준
    if sampleRate == 5e6
        cbw = 'CBW5';
    elseif sampleRate == 10e6
        cbw = 'CBW10';
    elseif sampleRate == 20e6
        cbw = 'CBW20';
    elseif sampleRate == 40e6
        cbw = 'CBW40';
    elseif sampleRate == 80e6
        cbw = 'CBW80';
    elseif sampleRate == 160e6
        cbw = 'CBW160';
    elseif sampleRate == 320e6
        cbw = 'CBW320';
    else
        error('지원하지 않는 sampleRate입니다');
    end

end



function [cpLen, symLen] =  extractLLTFLengths(sampleRate)
    cpDuration  = 1.6e-6;  % 1.6 us
    symDuration = 3.2e-6;  % 3.2 us
    cpLen  = round(cpDuration  * sampleRate);
    symLen = round(symDuration * sampleRate);
end



function [newBuffer, nextIdx] = advanceBuffer(buffer, processedEnd, cbw, threshold, sampleRate)
    % overlap으로 패킷이 겹쳐 있거나 바로 뒤에 붙어있을 경우를 대비 (20us 정도)
    overlap = round(20e-6 * sampleRate); 
    
    % 다음 시작점 계산
    nextStart = processedEnd - overlap;
    
    if nextStart < 1, nextStart = 1; end
    
    if nextStart < size(buffer, 1)
        newBuffer = buffer(nextStart:end, :);
        % 밀어낸 버퍼에서 바로 다음 패킷 탐지
        nextIdx = wlanPacketDetect(newBuffer, cbw, 0, threshold);
    else
        newBuffer = [];
        nextIdx = [];
    end
end