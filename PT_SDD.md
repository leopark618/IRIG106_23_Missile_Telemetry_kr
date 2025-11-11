# 미사일 텔레메트리 시스템 - 설명서

##  목차
1. [프로젝트 개요](#프로젝트-개요)
2. [변수 프리픽스 시스템](#변수-프리픽스-시스템)
3. [자료 구조 (Struct) 완전 설명](#자료-구조-struct-완전-설명)
4. [파일별 상세 가이드](#파일별-상세-가이드)
5. [변수 변경 효과](#변수-변경-효과)
6. [전체 데이터 흐름](#전체-데이터-흐름)

---

# 프로젝트 개요

##  미사일 텔레메트리 시스템이란?

**목적**: 비행 중인 미사일에서 센서 데이터를 수집 → 처리 → 암호화 → RF 신호로 변조 → 송신

**주요 기능**:
-  IMU, 압력, 온도 센서 데이터 수집
-  LDPC 오류 정정 코드 적용
-  SOQPSK 변조를 이용한 RF 신호 생성
-  발사 감지 및 텔레메트리 자동 전송
-  IRIG 106-23 표준 준수

**표준**: IRIG 106-23 (공군 계측 표준)

---

# 변수 프리픽스 시스템

##  3가지 프리픽스로 변수 분류

### **PT_ (Project Tuning) - 자유롭게 변경 가능 **

```
의미: 프로젝트별 튜닝 파라미터
변경: 자유롭게 변경 가능
범위: 각 변수마다 권장 범위 있음
목적: 항공기별 맞춤 성능 조정

예시:
PT_SENSOR_SAMPLE_PERIOD_MS = 1         (센서 샘플링)
PT_LAUNCH_ACCEL_THRESHOLD_G = 5.0f     (발사 감지)
PT_LDPC_DECODER_MAX_ITERATIONS = 50    (오류 정정)
PT_PLL_BANDWIDTH_SCALE = 0.01f         (위상 추적)
```

**장점**:
- 다양한 환경에 최적화 가능
- 성능 튜닝 용이
- 실시간 조정 가능

**단점**:
- 잘못 설정하면 성능 저하
- 지상국과 독립적으로 변경 가능

---

### **IRIG_ (IRIG 106 표준) - 신중하게 변경 **

```
의미: IRIG 106 표준에서 권장하는 파라미터
변경: 신중하게, 지상국과 반드시 동기화!
범위: 표준 범위 내에서 선택
목적: 지상국과의 호환성 유지

예시:
IRIG_CARRIER_FREQ_HZ = 2.35e9          (반송파 주파수)
IRIG_DATA_RATE_BPS = 10e6              (데이터율)
IRIG_LDPC_CODE_RATE = LDPC_RATE_2_3    (부호율)
```

**주의사항**:
-  변경 시 지상국 수신기도 동일하게 변경 필수!
-  채널 대역폭 제약 고려
-  통신 거리에 영향

---

### **IRIGFIX_ (IRIG 106 고정) - 절대 변경 금지 **

```
의미: IRIG 106에서 고정으로 정한 값
변경: 절대 변경 금지!
이유: 표준 위배 + 호환성 상실 + 무선 규격 위반
결과: 변경 시 표준 비준수 시스템

예시:
IRIGFIX_LDPC_N = 8192                  (LDPC 길이)
IRIGFIX_CPM_RHO = 0.70                 (CPM 매개변수)
IRIGFIX_CPM_B = 1.25                   (CPM 매개변수)
IRIGFIX_SAMPLE_RATE = 80e6             (샘플링 레이트)
```

**변경하면 안되는 이유**:
-  IRIG 106-23 표준 정의
-  암호/복호 불일치 → 통신 불가
-  무선 규격 위반 (FCC 등)
-  지상국과 호환 불가

---

# 자료 구조 (Struct) 완전 설명

## 1️ float_complex (복소수)

### 정의
```c
typedef struct {
    float real;          /* 실수부 (In-phase, I) */
    float imag;          /* 허수부 (Quadrature, Q) */
} float_complex;
```

### 의미
- **목적**: RF 신호를 I/Q 신호로 표현
- **real**: 동위상 성분 = cos(θ)
- **imag**: 직교 성분 = sin(θ)

### 예시
```
SOQPSK 심볼 = I + j*Q = 0.877 + j*0.479
              ↓
float_complex signal;
signal.real = 0.877;
signal.imag = 0.479;
```

---

## 2️ SOQPSK_Modulator (변조기)

### 정의
```c
typedef struct {
    float carrier_freq;         /* 반송파 (Hz) - IRIG_CARRIER_FREQ_HZ */
    float sample_rate;          /* 샘플링 (Hz) - IRIGFIX_SAMPLE_RATE */
    int samples_per_symbol;     /* 심볼당 샘플 - IRIGFIX_SAMPLES_PER_SYMBOL */
    
    float *frequency_pulse;     /* CPM 필터 배열 (크기: pulse_length) */
    int pulse_length;           /* 펄스 길이 = 8 * samples_per_symbol = 64 */
    
    float phase_accum;          /* 누적 위상 (라디안, -π ~ π)
                                 * 이전 심볼 위상을 유지해서 연속성 보장
                                 */
} SOQPSK_Modulator;
```

### 동작
```
비트 입력 [0,1,0,1]
    ↓
CPM 변조 (frequency_pulse 적용)
    ↓
위상 누적 (phase_accum 사용)
    ↓
반송파 실림 (carrier_freq)
    ↓
I/Q 신호 출력 [float_complex]
```

### 파라미터
| 파라미터 | 값 | 변수명 | 변경 |
|---------|-----|-------|------|
| 반송파 | 2.35 GHz | IRIG_CARRIER_FREQ_HZ | ⚠️ 신중 |
| 샘플링 | 80 MHz | IRIGFIX_SAMPLE_RATE | ❌ 금지 |
| 심볼당샘플 | 8 | IRIGFIX_SAMPLES_PER_SYMBOL | ❌ 금지 |
| CPM ρ | 0.70 | IRIGFIX_CPM_RHO | ❌ 금지 |
| CPM B | 1.25 | IRIGFIX_CPM_B | ❌ 금지 |

---

## 3️ SOQPSK_Demodulator (복조기)

### 정의
```c
typedef struct {
    /* === RF 파라미터 === */
    float carrier_freq;         /* 반송파 주파수 */
    float sample_rate;          /* 샘플링 레이트 */
    int samples_per_symbol;     /* 심볼당 샘플 */
    
    /* === PLL (Phase Lock Loop) === */
    float pll_phase;            /* 현재 위상 (라디안)
                                 * 범위: -π ~ π
                                 * 동작: 수신 신호 위상 추적
                                 */
    
    float pll_freq;             /* 주파수 오차 (rad/s)
                                 * 목적: 도플러 시프트 추정
                                 * 예: 미사일 고속 이동 시 반송파 변함
                                 */
    
    float loop_bw;              /* 루프 대역폭
                                 * 계산: symbol_rate * PT_PLL_BANDWIDTH_SCALE
                                 * 작을수록: 안정적 (느린 도플러)
                                 * 클수록: 민감 (빠른 도플러)
                                 */
    
    float damping;              /* 댐핑 계수
                                 * 권장: 0.707 (임계 감쇠)
                                 * 변수명: PT_PLL_DAMPING_FACTOR
                                 */
    
    /* === 타이밍 복구 === */
    float timing_mu;            /* 타이밍 오프셋 (0.0~1.0)
                                 * 0.5: 심볼 중간 (최적)
                                 * 목적: 정확한 샘플링 위치 찾기
                                 */
    
    float timing_error;         /* 타이밍 에러 신호
                                 * 계산: (late - early) * mid
                                 * 용도: timing_mu 업데이트
                                 */
    
    /* === 비터비 디코더 === */
    uint8_t current_state;      /* 현재 상태 (0~7, 3비트)
                                 * 예: 5 = (1,0,1) in binary
                                 */
    
    float path_metrics[8];      /* ✅ 배열! (8개 상태)
                                 * path_metrics[s] = 상태 s까지의 누적 거리
                                 * 동작: 매 심볼마다 업데이트
                                 * 최종: 가장 작은 메트릭 선택
                                 */
} SOQPSK_Demodulator;
```

### 복조 파이프라인
```
I/Q 신호 입력
    ↓
PLL (Phase Lock Loop)
├─ pll_phase: 반송파 위상 추적
├─ pll_freq: 도플러 보정
└─ loop_bw, damping: 응답 특성
    ↓
타이밍 복구 (Symbol Timing Recovery)
├─ timing_mu: 샘플 위치 조정
└─ timing_error: 피드백
    ↓
비터비 디코더 (Viterbi)
├─ current_state: 현재 상태
└─ path_metrics[8]: 경로 메트릭
    ↓
비트 출력
```

---

## 4️ LDPC_Encoder / LDPC_Decoder

### 정의
```c
typedef struct {
    LDPC_CodeRate rate;         /* 부호율: 1/2, 2/3, 4/5 */
    
    int K;                      /* 정보 비트 개수
                                 * 1/2: K=4096
                                 * 2/3: K=5461 (권장)
                                 * 4/5: K=6554
                                 */
    
    int N;                      /* 코드워드 길이: 8192 (고정)
                                 * 변수명: IRIGFIX_LDPC_N
                                 */
    
    int M;                      /* 패리티 비트 개수: M = N - K */
    
    int8_t **proto_matrix;      /* 프로토타입 행렬 (2D 배열)
                                 * 크기: proto_rows × proto_cols
                                 */
    
    int proto_rows;             /* = M / 128 */
    int proto_cols;             /* = N / 128 */
} LDPC_Encoder / LDPC_Decoder;
```

### 인코딩 과정
```
정보 비트 (K = 5461)
    ↓
프로토타입 행렬 × 비트
    ↓
패리티 비트 계산 (M = 2731)
    ↓
코드워드 (N = 8192) = 정보(5461) + 패리티(2731)
```

### 디코딩 과정
```
수신 LLR (soft bits)
    ↓
반복 처리 (PT_LDPC_DECODER_MAX_ITERATIONS 번)
├─ 1. 체크 노드 처리
├─ 2. 변수 노드 처리
└─ 3. 메시지 업데이트
    ↓
조기 종료 판정 (PT_LDPC_EARLY_TERMINATION)
├─ 패리티 확인: 모두 0인가?
├─ YES → 종료 (평균 30% 가속)
└─ NO → 계속 반복
    ↓
하드 결정: LLR > 0 ? 비트0 : 비트1
    ↓
정보 비트 (K = 5461)
```

---

# 파일별 상세 가이드

##  include/missile_telemetry.h

### 역할
- 시스템 최상위 구조 정의
- 모든 센서 데이터 프레임 정의
- 텔레메트리 시스템 상태 관리

### 주요 상수
```c
IRIGFIX_NUM_IMU_CHANNELS = 6           /* IMU 6축 */
IRIGFIX_NUM_PRESSURE_CHANNELS = 4      /* 압력 센서 4개 */
IRIGFIX_NUM_TEMP_CHANNELS = 8          /* 온도 센서 8개 */
IRIGFIX_NUM_GUIDANCE_CHANNELS = 16     /* 유도 명령 채널 */
```

### 주요 구조
```
MissileTelemetryFrame: 센서 데이터 한 세트
├─ frame_counter: 프레임 번호
├─ timestamp_us: 타임스탬프
├─ IMU: accel_x/y/z, gyro_x/y/z
├─ 센서: pressure[4], temperature[8]
├─ 제어: guidance_cmd[16], actuator_pos[16]
└─ GPS: latitude, longitude, altitude

MissileTelemetrySystem: 시스템 상태
├─ current_frame: 현재 프레임
├─ ldpc_encoder, soqpsk_modulator: 코덱
└─ system_armed, launch_detected: 상태 플래그
```

---

##  include/ldpc_codec.h

### 역할
- LDPC 오류 정정 코드 정의
- 3가지 부호율 지원
- 랜더마이저 및 ASM 정의

### 부호율별 특성
```
LDPC_RATE_1_2 (1/2)
├─ K = 4096, N = 8192
├─ 비율 = 50%
├─ 오류 정정: 가장 강함 ⭐⭐⭐
├─ 속도: 느림
└─ 사용: 극한 환경 (매우 낮은 SNR)

LDPC_RATE_2_3 (2/3) ⭐ 권장
├─ K = 5461, N = 8192
├─ 비율 ≈ 67%
├─ 오류 정정: 중간 (적절함) ⭐⭐
├─ 속도: 중간
└─ 사용: 일반적인 미사일 텔레메트리

LDPC_RATE_4_5 (4/5)
├─ K = 6554, N = 8192
├─ 비율 ≈ 80%
├─ 오류 정정: 약함 ⭐
├─ 속도: 빠름
└─ 사용: 높은 SNR 환경
```

---

##  include/soqpsk.h

### 역할
- SOQPSK 변조/복조 정의
- I/Q 신호 구조
- CPM 매개변수 정의

### 고정 상수
```c
IRIGFIX_CPM_RHO = 0.70        /* 스무딩 인자 */
IRIGFIX_CPM_B = 1.25          /* 대역폭 인자 */
IRIGFIX_CPM_T1 = 1.5          /* 시간 파라미터 1 */
IRIGFIX_CPM_T2 = 0.50         /* 시간 파라미터 2 */
```

---

##  src/7_main_integration.c

### 역할
- 모든 PT_/IRIG_/IRIGFIX_ 변수 정의
- 시스템 초기화 및 실행
- 메인 제어 로직

### PT_ 변수들
```c
PT_SENSOR_SAMPLE_PERIOD_MS = 1
PT_DATA_TX_PERIOD_MS = 10
PT_LAUNCH_ACCEL_THRESHOLD_G = 5.0f
PT_LAUNCH_SUSTAINED_SAMPLES = 10
PT_PLL_BANDWIDTH_SCALE = 0.01f
PT_PLL_DAMPING_FACTOR = 0.707f
PT_LDPC_DECODER_MAX_ITERATIONS = 50
PT_LDPC_EARLY_TERMINATION_ENABLE = 1
PT_TX_POWER_W = 3.0f
```

---

# 변수 변경 효과

##  PT_LDPC_DECODER_MAX_ITERATIONS 변경

```
기본값: 50 반복

변경 시나리오 1: 10 반복
├─ 처리 속도: 5배 빠름 ✅
├─ BER: 약간 증가 ❌
├─ CPU: 매우 낮음 ✅
└─ 사용: 실시간 필수, 높은 SNR

변경 시나리오 2: 30 반복
├─ 처리 속도: 약간 빠름 ✅
├─ BER: 거의 동일 ✅
├─ CPU: 낮음 ✅
└─ 사용: 실무 표준 (권장) ⭐

변경 시나리오 3: 50 반복 (기본)
├─ 처리 속도: 표준 ✅
├─ BER: 좋음 ✅
├─ CPU: 중간
└─ 사용: 일반적인 미사일

변경 시나리오 4: 100 반복
├─ 처리 속도: 2배 느림 ❌
├─ BER: 약 1dB 개선 ✅
├─ CPU: 높음 ❌
└─ 사용: 극도로 나쁜 채널
```

---

##  PT_PLL_BANDWIDTH_SCALE 변경

```
기본값: 0.01 (심볼 레이트의 1%)

변경 시나리오 1: 0.001 (매우 낮음)
├─ 안정성: 매우 높음 ✅
├─ 도플러 추적: 낮은 도플러에만 강함 ⚠️
├─ 사용: 천천히 움직이는 대상
└─ 예: 정지 상태, 저속 이동

변경 시나리오 2: 0.01 (표준) ⭐ 권장
├─ 안정성: 높음 ✅
├─ 도플러 추적: 중간 도플러 추적 가능 ✅
├─ 사용: 일반적인 미사일 (권장)
└─ 예: 발사 초기 ~ 중기

변경 시나리오 3: 0.05 (높음)
├─ 안정성: 중간 ⚠️
├─ 도플러 추적: 높은 도플러 추적 ✅
├─ 사용: 고속 미사일
└─ 예: 발사 직후 (최대 가속)

변경 시나리오 4: 0.1 (매우 높음)
├─ 안정성: 낮음 (노이즈 민감) ❌
├─ 도플러 추적: 매우 빠른 도플러 ✅
├─ 사용: 극도로 빠른 대상만
└─ 예: 초음속 미사일
```

---

##  PT_LAUNCH_ACCEL_THRESHOLD_G 변경

```
기본값: 5.0 G

변경 시나리오 1: 2.0 G (매우 민감)
├─ 감지 속도: 매우 빠름 ✅
├─ 오감지 위험: 높음 ❌
├─ 사용: 테스트 환경
└─ 문제: 진동, 노이즈로 오작동

변경 시나리오 2: 5.0 G (기본) ⭐ 권장
├─ 감지 속도: 정상 ✅
├─ 오감지 위험: 낮음 ✅
├─ 사용: 일반 미사일 (권장)
└─ 이유: 실제 발사 가속도 범위

변경 시나리오 3: 8.0 G (덜 민감)
├─ 감지 속도: 지연 ⚠️
├─ 오감지 위험: 매우 낮음 ✅
├─ 사용: 지진 지역 등 높은 진동
└─ 문제: 발사 감지 지연

변경 시나리오 4: 10.0 G (매우 둔감)
├─ 감지 속도: 매우 지연 ❌
├─ 오감지 위험: 거의 없음 ✅
├─ 사용: 거의 안 함
└─ 문제: 발사 감지 실패 가능
```

---

# 전체 데이터 흐름

##  완전한 송신 파이프라인

```
1️ 센서 수집 (PT_SENSOR_SAMPLE_PERIOD_MS마다)
   ├─ IMU: accel_x/y/z (±100 G)
   ├─ IMU: gyro_x/y/z (±2000 dps)
   ├─ 압력: 4개 채널 (PSI)
   ├─ 온도: 8개 채널 (°C)
   ├─ GPS: lat/lon/alt
   └─ 배터리 전압

2️ MissileTelemetryFrame 생성
   └─ 모든 센서 데이터 한 세트 패킹

3️ CRC-16 계산 및 추가
   └─ 오류 검출용

4️ 발사 감지 (PT_LAUNCH_ACCEL_THRESHOLD_G)
   ├─ 가속도 크기 계산: √(ax² + ay² + az²)
   ├─ PT_LAUNCH_SUSTAINED_SAMPLES 동안 확인
   └─ 조건 만족 → telemetry_active = true

5️ 데이터 전송 (PT_DATA_TX_PERIOD_MS마다)
   ├─ 프레임을 K=5461 비트로 변환
   ├─ LDPC 인코딩 (K 입력 → N=8192 코드워드)
   │  └─ PT_LDPC_DECODER_MAX_ITERATIONS, PT_LDPC_EARLY_TERMINATION
   ├─ 랜더마이저 (0/1 균등 분산)
   │  └─ LFSR seed: 0xACE1
   ├─ ASM 추가 (동기 마커: 0x1ACFFC1D)
   │  └─ 64비트 + 8192비트 = 8256비트
   ├─ SOQPSK 변조
   │  ├─ CPM 필터: IRIGFIX_CPM_RHO, IRIGFIX_CPM_B
   │  ├─ 반송파: IRIG_CARRIER_FREQ_HZ = 2.35 GHz
   │  ├─ 샘플링: IRIGFIX_SAMPLE_RATE = 80 MHz
   │  └─ 심볼당 샘플: 8
   ├─ I/Q 신호 생성 (float_complex)
   └─ RF 드라이버로 전송 (PT_TX_POWER_W = 3.0 W)

   ========== 채널 통과 (잡음 + 도플러 + 페이딩) ==========

6️ 지상국 수신 (수신기)
   ├─ RF 신호 수신 (I/Q 신호)
   ├─ SOQPSK 복조
   │  ├─ PLL 루프 (PT_PLL_BANDWIDTH_SCALE, PT_PLL_DAMPING_FACTOR)
   │  │  └─ 반송파 제거 + 도플러 추적
   │  ├─ 타이밍 복구
   │  │  └─ 정확한 심볼 샘플링 위치 찾기
   │  ├─ 비터비 디코더
   │  │  └─ 최적 경로 찾기 (path_metrics[8])
   │  └─ 소프트 비트 (LLR) 생성
   ├─ ASM 검출 및 동기
   ├─ LDPC 복호화
   │  ├─ 메시지 패싱 (체크/변수 노드)
   │  ├─ 조기 종료 판정 (패리티 확인)
   │  └─ 정보 비트 복원 (K=5461)
   └─ 센서 데이터 복원

7️ 텔레메트리 저장/분석
   └─ 비행 중 모든 센서 데이터 기록
```

---

##  각 모듈의 역할

| 모듈 | 입력 | 출력 | 목적 |
|------|------|------|------|
| **센서 수집** | 하드웨어 | MissileTelemetryFrame | 데이터 수집 |
| **CRC** | 프레임 | 체크섬 | 오류 검출 |
| **LDPC 인코더** | K비트 | N비트 | 오류 정정 코드 추가 |
| **랜더마이저** | N비트 | N비트 | 0/1 균등화 |
| **ASM** | N비트 | 64+N비트 | 동기 마커 추가 |
| **SOQPSK 변조** | 비트 | I/Q 신호 | RF 신호 생성 |
| **RF 드라이버** | I/Q | 전자기파 | 송신 |
| ⬇ **채널** ⬇ |
| **SOQPSK 복조** | I/Q신호 | LLR | RF 신호 분석 |
| **LDPC 복호** | LLR | K비트 | 오류 정정 & 복원 |
| **역랜더마이저** | N비트 | N비트 | 0/1 복원 |
| **ASM 검출** | 비트 | 동기 | 프레임 경계 찾기 |
| **CRC 확인** | 프레임 | OK/NG | 오류 확인 |

---

##  개발 가이드

### 초기 설정 (모두 기본값 사용)
```
1. 모든 PT_ = 기본값
2. 모든 IRIG_ = 표준값
3. 모든 IRIGFIX_ = 고정값 (변경 금지)
→ 컴파일 및 기본 기능 검증
```

### 환경별 튜닝 (PT_ 조정)

**지상 테스트**:
```
PT_SENSOR_SAMPLE_PERIOD_MS = 5      (전력 절약)
PT_DATA_TX_PERIOD_MS = 50           (느린 전송)
PT_LDPC_DECODER_MAX_ITERATIONS = 30 (빠른 처리)
```

**고속 미사일**:
```
PT_PLL_BANDWIDTH_SCALE = 0.05       (높은 도플러)
PT_LDPC_DECODER_MAX_ITERATIONS = 75 (강한 정정)
```

**극한 환경**:
```
PT_LAUNCH_ACCEL_THRESHOLD_G = 3.0   (민감하게)
PT_LDPC_DECODER_MAX_ITERATIONS = 100 (최강 정정)
IRIG_LDPC_CODE_RATE = LDPC_RATE_1_2  (강한 부호)
```

### 지상국 연동 (IRIG_ 조정)
```
 IRIG_값 변경 시:
1. 채널 변경: IRIG_CARRIER_FREQ_HZ
2. 데이터율 변경: IRIG_DATA_RATE_BPS
3. 부호율 변경: IRIG_LDPC_CODE_RATE
→ 반드시 지상국도 동일하게 변경!
```

### 프로덕션 (절대 하지 말 것)
```
❌ IRIGFIX_ 변경 금지
❌ 표준 벗어난 값 사용 금지
❌ 지상국과 불일치 금지
✅ 정기적인 성능 검증
✅ 비행 전 시스템 자가진단
```
