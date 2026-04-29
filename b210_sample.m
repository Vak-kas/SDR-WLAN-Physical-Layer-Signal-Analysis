if exist('rx','var')
    release(rx);
    clear rx;
end
clear;
clc;

%% 기기 연결 확인
radios = findsdru();


%% 초기 변수 설정
frequencyBand = 2.4;
channelNumber = 1;
captureTime = milliseconds(100); % 1000 = 1초
justOne = true; % 동작 모드 변경
%captureTime = 1;
sampleRate =  20; %샘플레이트 (채널 밴드)
decimationFactor = 1; %다운샘플링 비율
samplePerFrame = 200000; %한 번에 몇 개 샘플을 가져오는가(묶음 크기)
gain = 20; %RF신호를 얼마나 증폭해서 받을 것인가 (dB)




%% 초기 initial
radio = radios(1);
fc = wlanChannelFrequency(channelNumber, frequencyBand);
sampleRate = sampleRate * 1e6;


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



%% 신호 수신 및 실시간 감지 루프
disp("Scanning for Wi-Fi signals...");


fig = figure(1);
maxLoops = 200;
packetCount = 0;
stopAll = false;

buffer = [];

for i = 1:maxLoops
    
    if stopAll
        break;
    end

    [data, len] = rx();

    if len <=0
        continue;
    end
    
    % buffer 누적
    buffer = [buffer; data];
    
    % buffer 너무 커지는 것 방지
    if size(buffer,1) > 5e5
        buffer = buffer(end-3e5:end,:);
    end
    

    % 신호 수신 및 처리 루프
    disp("Receiving signals...");
    idx = wlanPacketDetect(buffer, sampleRateToCBW(sampleRate));
    
    while ~isempty(idx)
        
        packetCount = packetCount + 1;
        fprintf("패킷 발견 #%d (loop %d, idx %d)\n", packetCount, i, idx);
        
        rxPacket = buffer(idx:end,:);


        % --- 시각화 범위 통일 (8000 샘플 = 400μs) ---
        displayLimit = 8000; 
        set(0, 'CurrentFigure', fig);
        
        % 1번 칸: 시간 영역 그래프
        subplot(2,1,1); 
        t = (0:displayLimit-1) / sampleRate * 1e6; % μs 단위 계산

        plot(t, abs(rxPacket(1:displayLimit)));
        xlabel('Time (μs)');
        ylabel('Amplitude');
        title(sprintf('Time Domain - Packet #%d', packetCount));
        xlim([0 t(end)]); % 시간축 범위 고정
        grid on;

        % 2번 칸: 주파수 영역 (스펙트로그램)
        subplot(2,1,2);
        % 입력 데이터를 위와 똑같이 1:displayLimit로 제한
        spectrogram(rxPacket(1:displayLimit), 256, 200, 256, sampleRate, 'centered', 'yaxis');
        
        % 스펙트로그램 가로축 단위를 μs로 보기 좋게 제목 수정
        title('Frequency Domain (Spectrogram matching Time Domain)');
        
        drawnow;

        % ------------------------------------------
        %  1회용 캡처 여부
        if justOne
            stopAll = true;
            break;
        end
        
        skip = 2000;
        if idx + skip < size(buffer,1)
            buffer = buffer(idx + skip:end,:);
        else
            buffer = [];
        end
        
        idx = wlanPacketDetect(buffer, sampleRateToCBW(sampleRate));
        
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