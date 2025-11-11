# 미사일 텔레메트리 시스템 
## Software Design Description (SDD)
### DO-178C DAL-A 준수 문서

---

## 문서 정보

| 항목 | 내용 |
|------|------|
| **프로젝트명** | 미사일 텔레메트리 시스템  |
| **DAL (Design Assurance Level)** | DAL-A (최고 수준) |
| **표준** | DO-178C, IRIG 106-23 |
| **버전** | 3.0 |
| **작성일** | 2025-11-11 |
| **상태** | 설계 완료 |

---

## 1. 설계 개요

### 1.1 아키텍처

```
┌─────────────────────────────────────────────────────────────┐
│                    미사일 탑재 시스템                         │
├─────────────────────────────────────────────────────────────┤
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐      │
│  │   센서       │  │   카메라      │  │   제어       │      │
│  │  모듈        │  │   모듈        │  │   명령       │      │
│  └──────┬───────┘  └──────┬───────┘  └──────┬───────┘      │
│         │                 │                 │              │
│         └─────────────────┼─────────────────┘              │
│                           │                                │
│         ┌─────────────────▼────────────────┐               │
│         │     메인 통합 (main_integration)  │               │
│         └─────────────────┬────────────────┘               │
│                           │                                │
│         ┌─────────────────┼─────────────────┐              │
│         │                 │                 │              │
│    ┌────▼────┐    ┌───────▼──┐     ┌────────▼──┐          │
│    │LDPC코덱 │    │ SOQPSK   │     │데이터      │          │
│    │(Encoder/│    │(변조/    │     │저장        │          │
│    │Decoder) │    │복조)     │     │(메모리/SD) │          │
│    └────┬────┘    └───────┬──┘     └───────┬────┘          │
│         │                 │                │               │
│         └─────────────────┼────────────────┘               │
│                           │                                │
│         ┌─────────────────▼────────────────┐               │
│         │    긴급 시스템 + 설정 변경         │               │
│         └─────────────────┬────────────────┘               │
│                           │                                │
│                      RF 송신 (2.35 GHz)                    │
│                           │                                │
└───────────────────────────┼────────────────────────────────┘
                            │
                            ▼
                      ┌────────────┐
                      │ 지상국      │
                      │ (수신/제어) │
                      └────────────┘
```

### 1.2 모듈 구조

| 모듈 | 파일 | 역할 | 상태 |
|------|------|------|------|
| 센서 수집 | src/1_sensor_acquisition.c | IMU/압력/온도/GPS/배터리 | 완료 |
| LDPC 인코더 | src/2_ldpc_encoder.c | 오류 정정 부호 생성 | 완료 |
| LDPC 디코더 | src/3_ldpc_decoder.c | 오류 정정 복호화 | 완료 |
| LDPC Randomizer | src/4_ldpc_randomizer.c | 비트 균등화 | 완료 |
| SOQPSK 변조 | src/5_soqpsk_modulator.c | 신호 변조 | 완료 |
| SOQPSK 복조 | src/6_soqpsk_demodulator.c | 신호 복조 | 완료 |
| 데이터 저장 | src/7_data_storage.c | 메모리/SD 저장 | 완료 |
| 카메라 | src/8_camera_interface.c | 영상 캡처/저장 | 완료 |
| 지상국 제어 | src/9_ground_control.c | 명령 수신/처리 | 완료 |
| 긴급 시스템 | src/10_emergency_system.c | 안전 모니터링 | 완료 |
| 설정 변경 | src/11_telemetry_config.c | 실시간 파라미터 조정 | 완료 |
| 메인 통합 | src/main_integration.c | 통합 + 루프 | 완료 |

---

## 2. 상세 설계

### 2.1 센서 수집 모듈 (src/1_sensor_acquisition.c)

#### 데이터 구조

```c
typedef struct {
    uint32_t frame_counter;              // 프레임 번호
    uint64_t timestamp_us;               // 마이크로초 타임스탬프
    
    // IMU (Inertial Measurement Unit)
    float accel_x_g;                     // X축 가속도 (G)
    float accel_y_g;                     // Y축 가속도
    float accel_z_g;                     // Z축 가속도
    float gyro_x_dps;                    // X축 각속도 (deg/s)
    float gyro_y_dps;                    // Y축 각속도
    float gyro_z_dps;                    // Z축 각속도
    
    // 압력 센서 (4개)
    float pressure_1_pa;                 // 센서 1 (Pa)
    float pressure_2_pa;                 // 센서 2
    float pressure_3_pa;                 // 센서 3
    float pressure_4_pa;                 // 센서 4
    
    // 온도 센서 (8개)
    float temperature_1_c;               // 센서 1 (°C)
    float temperature_2_c;               // 센서 2
    // ... (8개 총)
    
    // GPS
    double latitude;                     // 위도 (도)
    double longitude;                    // 경도 (도)
    float altitude_m;                    // 고도 (m)
    float velocity_ms;                   // 속도 (m/s)
    
    // 배터리
    float battery_voltage_v;             // 배터리 전압 (V)
    float battery_current_a;             // 배터리 전류 (A)
    
    // CRC
    uint16_t crc_checksum;               // 16-bit CRC
} MissileTelemetryFrame;
```

#### 샘플링 주기

```
PT_SENSOR_SAMPLE_PERIOD_MS = 1 ms (조정 가능: 1~100 ms)

루프 동작:
├─ 1ms: 센서 읽음 → frame_counter++
├─ 1ms: timestamp_us += 1000
└─ 10ms: 프레임 전송 (10개 프레임)
```

#### CRC 계산

```
방식: CRC-16 (XOR 누적)
초기값: 0x0000
다항식: 0xA001 (CRC-16-CCITT)
```

---

### 2.2 LDPC 부호 모듈 (src/2~4_ldpc_*.c)

#### 인코딩 파이프라인

```
정보 비트 (K=5461 bits)
    ↓
LDPC 인코더
    ├─ 생성 행렬 G (5461×8192)
    ├─ 프로토타입 행렬 × 입력
    └─ GF(2) 연산 (XOR)
    ↓
부호어 (N=8192 bits)
    ↓
Randomizer (LFSR)
    ├─ Seed: 0xACE1
    ├─ 다항식: x^16 + x^15 + x^13 + x^4 + 1
    └─ 비트 0/1 균등화
    ↓
최종 부호어 (8192 bits)
```

#### Rate 2/3 설명

```
Rate = K/N = 5461/8192 ≈ 2/3

의미:
├─ 정보 비트 2개 → 부호어 3개
├─ 오버헤드: 50%
└─ 오류 정정 능력: ≈ 3~4 bit error

SNR 성능:
├─ SNR > 3 dB: BER < 1e-5 (목표 달성)
├─ SNR = 0 dB: BER ≈ 1e-2
└─ SNR < -3 dB: 디코딩 실패
```

#### 디코딩 알고리즘

```
알고리즘: Sum-Product (Belief Propagation)

반복 횟수: PT_LDPC_DECODER_MAX_ITERATIONS = 50

조기 종료 조건:
├─ syndrome = 0 (모든 parity check 통과)
├─ 또는 반복 횟수 도달
└─ 예상 성능: 평균 5~10 반복 (50회 이내)

LLR (Log Likelihood Ratio):
├─ 입력: 소프트 비트 (연속값)
├─ 처리: 확률 로그값 계산
└─ 출력: 하드 비트 결정
```

---

### 2.3 SOQPSK 변조 모듈 (src/5~6_soqpsk_*.c)

#### 변조 파이프라인

```
부호어 (8192 bits)
    ↓
기호 매핑 (8192 bits → 1024 심볼)
    ├─ 심볼당 8 bits
    ├─ 심볼 레이트: 80 MHz / 8 = 10 Mbps
    └─ 심볼 간격: 100 ns
    ↓
CPM 필터 (Continuous Phase Modulation)
    ├─ BT (시간-대역폭곱): 0.5
    ├─ 필터 길이: 4 심볼
    └─ 펄스 형태: Gaussian
    ↓
위상 누적 (Phase Accumulation)
    ├─ θ(t) = 2π ∫ f(t) dt
    ├─ 초기 위상: 0
    └─ 연속 위상 보장
    ↓
반송파 변조
    ├─ 반송파: 2.35 GHz
    ├─ fc = 2.35 × 10^9 Hz
    └─ IQ 신호: I = cos(ωt + θ), Q = sin(ωt + θ)
    ↓
샘플링
    ├─ 샘플 레이트: 80 MHz
    ├─ 심볼당 8샘플
    └─ 총 샘플: 1024 심볼 × 8 = 8192 샘플
    ↓
IQ 신호 (16-bit I, 16-bit Q)
```

#### 복조 파이프라인

```
수신 신호 (I/Q 샘플 @80MHz)
    ↓
PLL (Phase-Locked Loop) [v3 추가]
    ├─ 대역폭: PT_PLL_BANDWIDTH_SCALE × 10kHz
    ├─ 감쇠: PT_PLL_DAMPING_FACTOR = 0.707
    └─ 위상 추적 오차 < 1°
    ↓
타이밍 복구 (Timing Recovery)
    ├─ Gardner 알고리즘
    ├─ 심볼 동기화 오차 < 1%
    └─ 샘플 재정렬
    ↓
비터비 디코더 (Viterbi Decoder)
    ├─ 경로 선택 (최대 우도)
    ├─ 상태: 2^4 = 16개
    └─ 메트릭: 유클리드 거리
    ↓
LLR 신호 (Soft Bits)
    ├─ -127 ~ +127 범위
    ├─ 음수: 비트 0 가능성
    └─ 양수: 비트 1 가능성
    ↓
ASM 검출 (Attached Sync Marker)
    ├─ 마커: 64-bit 고정 패턴
    ├─ 동기화 확인
    └─ 프레임 경계 결정
    ↓
LDPC 복호화 → 원본 데이터 복원
```

---

### 2.4 데이터 저장소 (src/7_data_storage.c)

#### LogBuffer 구조

```c
typedef struct {
    LogEntry *buffer;               // 엔트리 배열
    uint32_t buffer_count;          // 현재 엔트리 개수
    uint32_t buffer_capacity;       // 최대 용량 (10000)
    uint32_t write_position;        // 쓰기 위치 (순환)
    bool is_full;                   // 가득 찬 여부
} LogBuffer;

typedef struct {
    uint32_t entry_id;              // 일련번호
    uint64_t timestamp_us;          // 타임스탐프
    MissileTelemetryFrame telemetry;// 센서 데이터
    uint8_t last_command_type;      // 마지막 명령
    float last_thrust_cmd;          // 마지막 추력
    uint32_t camera_frame_id;       // 카메라 프레임 ID
    uint8_t *camera_data;           // 영상 데이터
    uint32_t camera_data_size;      // 영상 크기
} LogEntry;
```

#### 순환 큐 동작

```
초기 상태:
write_position = 0
buffer_count = 0

10ms마다:
├─ LogEntry 생성
├─ buffer[write_position]에 기록
├─ write_position = (write_position + 1) % 10000
├─ if (buffer_count < 10000): buffer_count++
└─ else: 기존 데이터 덮어씀 (is_full = true)

10000 엔트리 = 100초 분량 데이터
```

#### SD 카드 저장

```c
bool DataStorage_SaveToSD(LogBuffer *log, const char *filename)
{
    FILE *file = fopen(filename, "wb");
    
    // 매직 넘버 저장
    uint32_t magic = IRIGFIX_LOG_MAGIC = 0x43464700;
    fwrite(&magic, 4, 1, file);
    
    // 엔트리 개수
    uint32_t count = log->buffer_count;
    fwrite(&count, 4, 1, file);
    
    // 각 엔트리 저장
    for (int i = 0; i < count; i++) {
        fwrite(&log->buffer[i], sizeof(LogEntry), 1, file);
        
        // 카메라 영상 저장
        if (log->buffer[i].camera_data_size > 0) {
            fwrite(log->buffer[i].camera_data,
                   log->buffer[i].camera_data_size, 1, file);
        }
    }
    
    fclose(file);
    return true;
}
```

---

### 2.5 카메라 인터페이스 (src/8_camera_interface.c)

#### 카메라 사양

```
해상도: 320×240 픽셀
색 포맷: YUYV (4:2:2 크로미넌스 서브샘플링)
프레임 크기: 320 × 240 × 2 = 153,600 bytes
프레임률: 10 fps (조정 가능: 1~60 fps)
프레임 간격: 100 ms

메모리 요구:
├─ 1프레임: ~150 KB
├─ 1초 (10fps): ~1.5 MB
├─ 100초: ~150 MB
└─ 저장 제약 고려 필요
```

#### 캡처 루프

```
매 100ms마다:

CameraFrame *frame = Camera_CaptureFrame(g_camera);
├─ frame_id: 프레임 번호 (0, 1, 2, ...)
├─ data: 영상 버퍼 (150 KB)
├─ data_size: 153,600 bytes
├─ timestamp_us: 캡처 시간
└─ width=320, height=240

DataStorage_WriteCameraFrame(g_log_buffer, frame_id, data, data_size);
├─ 해당 LogEntry에 영상 포인터 저장
├─ camera_data = malloc(data_size)
├─ memcpy(camera_data, data, data_size)
└─ SD 카드 저장 시 함께 저장

Camera_ReleaseFrame(frame);
└─ 임시 메모리 해제
```

---

### 2.6 지상국 제어 (src/9_ground_control.c)

#### 제어 명령 형식

```
메시지 헤더: 0x4354 ("CT")

CMD_THRUST_UPDATE:
├─ 추력 범위: 0~100%
├─ 범위 검증: if (thrust < 0 || thrust > 100) reject
└─ 적용: 즉시 모터 제어

CMD_RUDDER_UPDATE:
├─ 러더 범위: -45~+45°
├─ 범위 검증: if (angle < -45 || angle > 45) reject
└─ 적용: 서보 모터 조종

CMD_TRAJECTORY_CHANGE:
├─ 목표 방향: 0~359°
├─ 목표 고도: 0~1000 m
├─ 범위 검증: 유효 범위 체크
└─ 적용: 항법 시스템 업데이트
```

#### 타임아웃 처리

```
GroundControl_CheckTimeout(&g_control_state):

if (time_now - last_command_time > 2000 ms) {
    ├─ is_command_valid = false
    ├─ current_thrust = 0%         // 안전 모드
    ├─ 자동 착륙 시작
    └─ 로그: "[TIMEOUT] 지상국 신호 상실"
}
```

#### CRC 검증

```
방식: XOR 누적

CRC 계산:
crc = 0
for (각 바이트) {
    crc ^= byte;
    for (8 비트) {
        if (crc & 0x0001) {
            crc = (crc >> 1) ^ 0xA001;
        } else {
            crc >>= 1;
        }
    }
}

검증:
if (message_crc != calculated_crc) {
    reject_command();
}
```

---

### 2.7 긴급 시스템 (src/10_emergency_system.c)

#### 모니터링 항목

```
1. 배터리 전압
   ├─ 조건: battery_voltage < 10.0V
   ├─ 빈도: 1Hz (1초마다 확인)
   └─ 조치: 안전 모드 + 착륙 명령

2. 내부 온도
   ├─ 조건: temperature > 85°C
   ├─ 빈도: 1Hz
   └─ 조치: 냉각 팬 최대 + 임무 중단

3. GPS 신호
   ├─ 조건: gps_lock == false (5초 이상)
   ├─ 빈도: 1초 체크
   └─ 조치: 마지막 알려진 위치 보유 + 수동 제어 대기

4. 최대 고도
   ├─ 조건: altitude > altitude_limit
   ├─ 빈도: 1Hz
   └─ 조치: 강제 하강 명령
```

#### 자폭 시퀀스

```
EMERGENCY_DESTRUCT 명령 수신:

1. 확인 단계 (2초)
   ├─ 명령 재확인
   ├─ CRC 재검증
   └─ 물리적 확인 신호 필요

2. 준비 단계
   ├─ 모든 센서 셧다운
   ├─ 추진체 제어 중단
   ├─ 카메라 저장 완료

3. 실행 단계
   ├─ 폭약 발화 신호 송신
   ├─ 백업 폭약 준비
   └─ 확인 신호 송신

4. 사후 처리
   ├─ 블랙박스 저장
   └─ 텔레메트리 종료
```

---

### 2.8 설정 실시간 변경 (src/11_telemetry_config.c)

#### 파라미터 등록

```c
TelemetryConfig_RegisterFloatParam(g_config,
    param_id,              // 0~99
    "PT_LAUNCH_ACCEL_THRESHOLD_G",
    5.0f,                  // 기본값
    1.0f,                  // 최솟값
    10.0f                  // 최댓값
);

TelemetryConfig_RegisterIntParam(g_config,
    param_id,
    "PT_SENSOR_SAMPLE_PERIOD_MS",
    1,                     // 기본값
    1,                     // 최솟값
    100                    // 최댓값
);
```

#### 메시지 형식

```
설정 변경 요청 (0x4346 "CF"):
├─ msg_header: 0x4346
├─ num_params: 변경할 파라미터 개수
├─ param_updates[]: 파라미터 배열
└─ crc: CRC-16

설정 응답 (0x4352 "CR"):
├─ msg_header: 0x4352
├─ num_params: 전체 파라미터 개수
├─ param_info[]: 모든 파라미터 현황
└─ crc: CRC-16
```

#### 적용 절차

```
TelemetryConfig_ProcessUpdateMessage(g_config, msg):
├─ 메시지 헤더 검증 (0x4346)
├─ CRC 검증
├─ 각 파라미터 범위 검증 (min/max)
├─ 유효하면: is_dirty = true
└─ 무효하면: 거절

TelemetryConfig_SyncToHardware(g_config):
├─ 모든 파라미터 순회
├─ is_dirty == true인 것만 적용
├─ 하드웨어 제어 신호 송신
└─ is_dirty = false
```

---

### 2.9 메인 통합 (src/main_integration.c)

#### 초기화 순서

```
main():
├─ 1. MissileTM_InitializeSystem()
│  ├─ g_tm_system malloc + memset
│  ├─ g_soqpsk_mod malloc + memset
│  ├─ g_soqpsk_demod malloc + memset
│  ├─ g_ldpc_encoder malloc + memset
│  ├─ g_ldpc_decoder malloc + memset
│  ├─ g_log_buffer = DataStorage_Init(10000)
│  ├─ g_camera = Camera_Init()
│  ├─ g_emergency_state = EmergencySystem_Init()
│  ├─ g_config = TelemetryConfig_Init()
│  ├─ 파라미터 등록 (8개)
│  └─ 제어 상태 초기화
│
├─ 2. MissileTM_MainLoop()
│  └─ [상세 아래]
│
└─ 3. MissileTM_ShutdownSystem()
   ├─ Camera_Stop/Destroy
   ├─ 모든 malloc 영역 free
   ├─ 모든 포인터 NULL 할당
   └─ 정상 종료
```

#### 메인 루프 동작

```
loop_count = 0
max_iterations = 100

while (loop_count < max_iterations) {
    loop_count++;
    
    // 1. 센서 업데이트 (1ms)
    if (g_tm_system) {
        g_tm_system->current_frame.frame_counter++;
        g_tm_system->current_frame.timestamp_us += 1000;
    }
    
    // 2. 발사 감지 (100ms)
    float accel_mag = sqrt(ax^2 + ay^2 + az^2);
    if (accel_mag > 5.0G && !launch_detected) {
        g_tm_system->launch_detected = true;
        g_tm_system->telemetry_active = true;
        printf("[LAUNCH] 발사 감지!\n");
    }
    
    // 3. 데이터 로깅 (10ms)
    if (telemetry_active && g_log_buffer) {
        LogEntry log_entry;
        log_entry.entry_id = g_log_buffer->buffer_count;
        log_entry.timestamp_us = g_tm_system->current_frame.timestamp_us;
        memcpy(&log_entry.telemetry, &g_tm_system->current_frame, ...);
        DataStorage_WriteEntry(g_log_buffer, &log_entry);
        g_frames_transmitted++;
    }
    
    // 4. 카메라 캡처 (선택)
    if (g_camera) {
        CameraFrame *frame = Camera_CaptureFrame(g_camera);
        if (frame) {
            DataStorage_WriteCameraFrame(...);
            Camera_ReleaseFrame(frame);
        }
    }
    
    // 5. 타임아웃 체크
    GroundControl_CheckTimeout(&g_control_state);
    EmergencySystem_CheckConditions();
    
    // 6. 긴급 상황 체크
    if (EmergencySystem_IsInEmergency(g_emergency_state)) {
        printf("[EMERGENCY] 긴급 모드 활성화\n");
        break;
    }
    
    // 7. 출력
    if (loop_count % 10 == 0) {
        printf("[%d ms] TX: %d, Log: %d\n",
               loop_count, g_frames_transmitted,
               DataStorage_GetEntryCount(g_log_buffer));
    }
}
```

---

## 3. DO-178C DAL-A 준수

### 3.1 메모리 안전성

| 요구사항 | 구현 |
|---------|------|
| NULL 체크 | 모든 malloc 후 if (!ptr) check |
| 메모리 누수 | free() + NULL 할당 |
| 버퍼 오버플로우 | strncpy + null terminator |
| 구조체 초기화 | memset() 100% |

### 3.2 입력 검증

| 요구사항 | 구현 |
|---------|------|
| CRC 검증 | 모든 메시지 CRC 확인 |
| 매직 넘버 | MSG_HEADER 0x4XXX 확인 |
| 범위 검증 | min_val ≤ value ≤ max_val |
| 타입 검증 | type 필드 확인 |

### 3.3 예외 처리

| 요구사항 | 구현 |
|---------|------|
| 세그먼트 폴트 | memset() 초기화 |
| 무한 루프 | max_iterations 설정 |
| 타임아웃 | 2초 제어 타임아웃 |
| 오류 복구 | 안전 모드 자동 전환 |

---

## 4. 컴파일 및 링크

### 4.1 Makefile

```makefile
CC = gcc
CFLAGS = -Wall -O2 -lm -Iinclude

SOURCES = src/1_sensor_acquisition.c \
          src/2_ldpc_encoder.c \
          src/3_ldpc_decoder.c \
          src/4_ldpc_randomizer.c \
          src/5_soqpsk_modulator.c \
          src/6_soqpsk_demodulator.c \
          src/7_data_storage.c \
          src/8_camera_interface.c \
          src/9_ground_control.c \
          src/10_emergency_system.c \
          src/11_telemetry_config.c \
          src/main_integration.c

OBJECTS = $(SOURCES:.c=.o)

all: missile_telemetry

missile_telemetry: $(OBJECTS)
	$(CC) -o $@ $^ $(CFLAGS)

%.o: %.c
	$(CC) -c $< -o $@ $(CFLAGS)

clean:
	rm -f $(OBJECTS) missile_telemetry
```

### 4.2 링크 순서

```
1. src/1_sensor_acquisition.o
2. src/2_ldpc_encoder.o
3. src/3_ldpc_decoder.o
4. src/4_ldpc_randomizer.o
5. src/5_soqpsk_modulator.o
6. src/6_soqpsk_demodulator.o
7. src/7_data_storage.o
8. src/8_camera_interface.o
9. src/9_ground_control.o
10. src/10_emergency_system.o
11. src/11_telemetry_config.o
12. src/main_integration.o

순서 무관: make가 자동 처리
```

---

## 5. 성능 특성

### 5.1 시간 특성

| 항목 | 값 | 단위 |
|------|-----|------|
| 센서 샘플링 | 1 | ms |
| 프레임 전송 | 10 | ms |
| 발사 감지 | 100 | ms |
| 카메라 캡처 | 100 | ms |
| 제어 타임아웃 | 2000 | ms |
| 설정 동기화 | 5000 | ms |

### 5.2 메모리 특성

| 항목 | 크기 |
|------|------|
| LogBuffer (10000) | 50~100 MB |
| CameraFrame (단일) | ~150 KB |
| 설정 파라미터 | ~50 KB |
| 스택 공간 | ~1 MB |
| **총 메모리** | **~200 MB** |

### 5.3 처리 성능

| 항목 | 값 |
|------|-----|
| LDPC 인코딩 | < 1 ms |
| LDPC 디코딩 | < 5 ms |
| SOQPSK 변조 | < 1 ms |
| 데이터 저장 | < 0.1 ms |

---

## 6. 테스트 전략

### 6.1 단위 테스트

```bash
각 모듈 독립적 테스트:
├─ LDPC: 부호어 생성/복호화
├─ SOQPSK: IQ 신호 생성/복원
├─ 데이터 저장: 10000 엔트리 기록/읽음
├─ 카메라: 프레임 캡처
├─ 제어: 명령 검증
├─ 긴급: 조건 감지
└─ 설정: 파라미터 변경
```

### 6.2 통합 테스트

```bash
시스템 전체 실행:
make clean
make
./missile_telemetry

예상 결과:
├─ 시스템 초기화 완료
├─ 발사 감지 (1ms)
├─ 100 루프 실행
├─ 100 엔트리 로깅
└─ 정상 종료
```

### 6.3 회귀 테스트

```
각 변경 후:
├─ 컴파일 성공
├─ 실행 시 오류 없음
├─ 메모리 누수 없음 (Valgrind)
└─ 출력 일치
```

---

## 7. 승인

| 역할 | 이름 | 서명 | 날짜 |
|------|------|------|------|
| 시스템 아키텍트 | - | - | 2025-11-11 |
| 소프트웨어 리드 | - | - | 2025-11-11 |
| 품질 보증 | - | - | 2025-11-11 |
| 프로젝트 매니저 | - | - | 2025-11-11 |

---

**END OF SDD**
