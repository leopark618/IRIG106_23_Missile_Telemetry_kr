# 미사일 텔레메트리 시스템 - 작동 플로우 (화살표 버전)

---

## 📊 1. 전체 송수신 흐름

```
[송신 측]

센서 (IMU, 압력, 온도, GPS, 배터리)
    ↓
MissileTelemetryFrame (센서 데이터 패킹)
    ↓
CRC 계산
    ↓
발사 감지 체크 (가속도 > 5.0G ?)
    → NO: 대기
    → YES: 텔레메트리 활성화
    ↓
비트 변환 (프레임 → K=5461 비트)
    ↓
LDPC 인코더 (K 비트 → N=8192 비트)
    ↓
랜더마이저 (0/1 균등화, LFSR)
    ↓
ASM 추가 (동기 마커 64비트)
    ↓
SOQPSK 변조 (비트 → I/Q 신호)
    ├─ CPM 필터
    ├─ 위상 누적
    └─ 반송파 2.35 GHz
    ↓
샘플링 (80 MHz, 심볼당 8샘플)
    ↓
RF 드라이버 (3.0W 송신)
    ↓
안테나 → 전자기파 송신


========== 채널 (잡음, 도플러, 페이딩) ==========


[수신 측]

안테나 ← 전자기파 수신
    ↓
RF 증폭기 → ADC
    ↓
I/Q 신호 (80 MHz 샘플링)
    ↓
SOQPSK 복조
    ├─ PLL 루프 (위상 추적)
    ├─ 타이밍 복구 (심볼 동기)
    └─ 비터비 디코더 (경로 선택)
    ↓
LLR 신호 (소프트 비트)
    ↓
ASM 검출 (동기 마커 찾기)
    ↓
LDPC 복호화 (반복 처리, 조기 종료)
    ↓
역랜더마이저 (0/1 원본 복원)
    ↓
CRC 확인 (OK? → 진행, NG? → 폐기)
    ↓
센서 데이터 복원
    ↓
저장/분석
```

---

## 📊 2. 송신 파이프라인 상세

```
[PT_DATA_TX_PERIOD_MS = 10ms 마다]

현재 프레임 읽음
    ↓
비트 변환 (프레임 → 5461 비트)
    ↓
LDPC 인코더
    ↓
정보 비트 5461개 입력
    ↓
프로토타입 행렬 × 입력 (GF(2) 연산)
    ↓
패리티 비트 2731개 계산
    ↓
코드워드 8192비트 출력
    ├─ 정보: 5461
    └─ 패리티: 2731
    ↓
랜더마이저 (LFSR 기반)
    ↓
Seed = 0xACE1, Poly = 0xB400
    ↓
코드워드[i] XOR LFSR_output[i]
    ↓
0과 1이 균등하게 분포된 8192비트
    ↓
ASM 추가
    ↓
64비트 동기 마커 (0x1ACFFC1D)
    ↓
[64비트 ASM] + [8192비트 코드워드] = 8256비트
    ↓
SOQPSK 변조기
    ↓
심볼별 처리 (8256개 심볼)
    ↓
CPM 필터 적용 (ρ=0.70, B=1.25)
    ↓
위상 누적 (연속성 유지)
    ↓
phase(n) = phase(n-1) + 주파수_변화
    ↓
반송파 실림 (2.35 GHz)
    ↓
I 성분 = cos(carrier_phase + phase)
Q 성분 = sin(carrier_phase + phase)
    ↓
I/Q 신호 생성 (8256 × 8 = 66048 샘플)
    ↓
각 샘플 (I[n], Q[n]) 순차 생성
    ↓
RF 드라이버로 전달
    ↓
디지털 → 아날로그 변환
    ↓
RF 증폭 (3.0W)
    ↓
안테나 → 송신
```

---

## 📊 3. 수신 파이프라인 상세

```
[수신 신호 처리]

안테나에서 I/Q 신호 수신 (66048 샘플)
    ↓
[Step 1] PLL (Phase Lock Loop)
    ↓
현재 위상 추정: pll_phase
    ↓
반송파 제거: baseband = received × local_osc
    ↓
에러 신호 계산: error = baseband_Q × sign(baseband_I)
    ↓
루프 필터 (2차)
    ├─ pll_freq += Ki × error
    └─ pll_phase += Kp × error + pll_freq
    ↓
다음 샘플로 반복
    ↓
결과: 위상이 정확히 추적된 베이스밴드 신호


[Step 2] 타이밍 복구
    ↓
현재 심볼 위치: n
    ↓
Early = 샘플[n-4], Mid = 샘플[n], Late = 샘플[n+4]
    ↓
타이밍 에러 = (Late - Early) × Mid
    ↓
timing_mu += K_t × error
    ↓
다음 심볼 위치 조정
    ↓
결과: 정확한 심볼 샘플링 위치


[Step 3] 비터비 디코더
    ↓
8개 상태(000~111)에 대해 거리 계산
    ↓
state[i] = min(이전상태 → 현재상태 비용)
    ↓
백트레킹으로 최적 경로 선택
    ↓
소프트 비트 LLR 생성
    ├─ LLR > 0: 비트 0 (확신도 = |LLR|)
    └─ LLR < 0: 비트 1 (확신도 = |LLR|)
    ↓
결과: 8192개 LLR 신호 (부동소수점)


[Step 4] ASM 검출
    ↓
슬라이딩 윈도우로 64비트 검색
    ↓
각 위치에서 표준 ASM과 비교
    ↓
오류 비트 < 2?
    ├─ NO: 계속 검색
    └─ YES: 동기 획득! (프레임 시작 위치)
    ↓
나머지 8192비트 추출


[Step 5] LDPC 디코더 (반복 처리)
    ↓
입력: 8192개 LLR
    ↓
반복 i=1 to PT_LDPC_DECODER_MAX_ITERATIONS (50)
    ↓
체크 노드 처리 (패리티 제약 적용)
    ↓
변수 노드 처리 (신뢰도 업데이트)
    ↓
메시지 전파
    ↓
조기 종료 판정
    ├─ 패리티 모두 만족? (H×x = 0)
    ├─ YES: 반복 종료!
    └─ NO: 계속 반복
    ↓
하드 결정: bit = LLR > 0 ? 0 : 1
    ↓
결과: 5461비트 정보 비트


[Step 6] 역랜더마이저
    ↓
동일한 LFSR (Seed = 0xACE1) 초기화
    ↓
각 비트: decoded[i] XOR LFSR_output[i]
    ↓
결과: 원본 정보 비트


[Step 7] CRC 확인
    ↓
CRC 계산
    ↓
계산값 == 수신값?
    ├─ YES: 정상! → 데이터 신뢰
    └─ NO: 오류 검출 → 프레임 폐기
    ↓
결과: 센서 데이터 복원 완료!
```

---

## 📊 4. 발사 감지 플로우

```
[1ms 주기]

센서에서 가속도 읽음 (ax, ay, az)
    ↓
크기 계산: mag = √(ax² + ay² + az²)
    ↓
비교: mag > PT_LAUNCH_ACCEL_THRESHOLD_G (5.0G)?
    ↓
NO (< 5.0G)
    ├─ counter = 0
    └─ 다음 반복으로
    ↓
YES (≥ 5.0G)
    ├─ counter++
    ↓
비교: counter ≥ PT_LAUNCH_SUSTAINED_SAMPLES (10)?
    ↓
NO (< 10)
    ├─ 다음 반복으로
    ↓
YES (≥ 10)
    ├─ 발사 감지!
    ├─ launch_detected = true
    ├─ telemetry_active = true
    └─ 텔레메트리 자동 시작
```

---

## 📊 5. PLL (위상 추적) 플로우

```
[매 샘플마다]

수신 신호: rx_I, rx_Q (복소수)
    ↓
로컬 진동자 생성
    ├─ cos_phase = cos(pll_phase)
    └─ sin_phase = sin(pll_phase)
    ↓
신호 × 로컬 진동 (복소수 곱셈)
    ├─ baseband_I = rx_I × cos - rx_Q × sin
    └─ baseband_Q = rx_Q × cos + rx_I × sin
    ↓
에러 신호 추출
    ├─ error = baseband_Q × (baseband_I > 0 ? 1 : -1)
    ↓
루프 필터 (2차 시스템)
    ├─ pll_freq = pll_freq + Ki × error
    │  (적분 항: 저주파 드리프 보정)
    ├─ pll_phase = pll_phase + Kp × error + pll_freq
    │  (비례 항: 빠른 응답)
    ↓
위상 정규화
    ├─ IF pll_phase > π: pll_phase -= 2π
    ├─ IF pll_phase < -π: pll_phase += 2π
    ↓
다음 샘플로 진행
    ↓
반복

[파라미터]
Kp = loop_bw / symbol_rate
Ki = (loop_bw / symbol_rate)²
loop_bw = symbol_rate × PT_PLL_BANDWIDTH_SCALE (0.01)
         = 1.25 MHz × 0.01 = 12.5 kHz
damping = PT_PLL_DAMPING_FACTOR (0.707: 임계 감쇠)
```

---

## 📊 6. 비터비 디코더 플로우

```
[매 심볼마다]

수신 심볼: symbol (복소수)
    ↓
8개 상태 각각에 대해 거리 계산
    ├─ 상태 0(000): dist[0] = ||symbol - const[0]||²
    ├─ 상태 1(001): dist[1] = ||symbol - const[1]||²
    ├─ ...
    └─ 상태 7(111): dist[7] = ||symbol - const[7]||²
    ↓
상태 전이 메트릭 계산
    ├─ new_metric[0] = min(path[0]→0, path[4]→0) + dist[0]
    ├─ new_metric[1] = min(path[0]→1, path[4]→1) + dist[1]
    ├─ new_metric[2] = min(path[1]→2, path[5]→2) + dist[2]
    ├─ new_metric[3] = min(path[1]→3, path[5]→3) + dist[3]
    ├─ new_metric[4] = min(path[2]→4, path[6]→4) + dist[4]
    ├─ new_metric[5] = min(path[2]→5, path[6]→5) + dist[5]
    ├─ new_metric[6] = min(path[3]→6, path[7]→6) + dist[6]
    └─ new_metric[7] = min(path[3]→7, path[7]→7) + dist[7]
    ↓
경로 메트릭 업데이트
    ├─ path_metrics[] = new_metric[]
    ↓
다음 심볼로
    ↓
마지막 심볼 후:
    ├─ min_metric = min(path_metrics[])
    ├─ final_state = argmin(path_metrics[])
    ↓
백트레킹 (최종 상태부터 처음까지 역순)
    ├─ 최적 경로 복원
    └─ 디코딩된 비트 시퀀스 생성
```

---

## 📊 7. LDPC 메시지 패싱 플로우

```
[LDPC 디코더 반복 처리]

반복 i=1
    ↓
변수 노드 처리
    ├─ 각 정보/패리티 비트에 대해
    ├─ 연결된 체크 노드에서 메시지 수신
    ├─ LLR + 모든 메시지 합산
    ├─ 새로운 가설 생성
    └─ 인접 체크 노드로 보낼 메시지 준비
    ↓
체크 노드 처리
    ├─ 각 패리티 제약에 대해
    ├─ 연결된 변수 노드에서 메시지 수신
    ├─ 패리티 비트 관계식 적용
    ├─ 수정 신호 계산
    └─ 인접 변수 노드로 보낼 메시지 준비
    ↓
메시지 교환 완료
    ↓
수렴 판정
    ├─ 현재 가설로 하드 결정
    ├─ H × x_hard = 0 인가? (패리티 확인)
    ├─ YES: 반복 종료 (성공!)
    └─ NO: 계속 반복
    ↓
반복 i++
    ↓
IF i > PT_LDPC_DECODER_MAX_ITERATIONS (50)
    ├─ 종료 (최대 반복 도달)
    └─ 최선의 가설 선택
    ↓
ELSE
    └─ 다시 변수 노드로
    ↓
최종 출력: 정보 비트 (5461개)
```

---

## 📊 8. 타이밍 복구 플로우

```
[매 심볼마다]

현재 심볼 인덱스: n
현재 타이밍 오프셋: timing_mu (0.0~1.0)
    ↓
신호 샘플 추출
    ├─ early = sample[n - 4]   (심볼 전반부)
    ├─ mid = sample[n]         (심볼 중앙)
    └─ late = sample[n + 4]    (심볼 후반부)
    ↓
타이밍 에러 계산
    ├─ error = (late - early) × mid
    ↓
에러 해석
    ├─ error > 0: 샘플링이 너무 늦음 → 더 앞으로
    ├─ error < 0: 샘플링이 너무 빠름 → 더 뒤로
    └─ error ≈ 0: 최적 위치 (유지)
    ↓
적응형 업데이트
    ├─ timing_mu = timing_mu + K_t × error
    │  (K_t: 타이밍 루프 이득, 보통 0.01~0.1)
    ↓
다음 심볼 위치 계산
    ├─ n_next = n + 8 + (int)timing_mu
    │  (8: samples_per_symbol)
    ├─ timing_mu = frac(timing_mu)
    │  (소수 부분만 유지)
    ↓
다음 심볼로 진행
```

---

## 📊 9. 시간 동작 (타이밍)

```
t = 0ms
    └─ 시스템 시작
       ├─ MissileTM_Create()
       ├─ LDPC, SOQPSK 초기화
       └─ Task 생성

t = 0~1ms
    └─ 센서 샘플링 주기 1
       ├─ IMU 읽음 (6축)
       ├─ 압력 읽음 (4개)
       ├─ 온도 읽음 (8개)
       └─ GPS 읽음

t = 1~2ms
    └─ 센서 샘플링 주기 2

...

t = 5ms
    └─ 발사 감지 체크 (100ms 주기에서)
       ├─ 가속도 크기 계산
       ├─ 임계값 비교 (> 5.0G?)
       └─ 조건 만족 시 신호

...

t = 10ms
    └─ 첫 번째 데이터 전송 (10ms 주기에서)
       ├─ 프레임 수집
       ├─ LDPC 인코딩 (~1ms)
       ├─ 랜더마이저 (~0.1ms)
       ├─ SOQPSK 변조 (~1.5ms)
       ├─ RF 전송 시작
       └─ 프레임 카운트: 1

t = 10~16ms
    └─ 첫 번째 프레임 송신 중
       ├─ 66048개 샘플 순차 전송
       └─ RF 전력: 3W

t = 20ms
    └─ 두 번째 데이터 전송
       └─ 프레임 카운트: 2

...반복...
```

---

## 📊 10. 에러 처리

```
센서 오류 감지
    ├─ 범위 밖? (-100G ~ +100G)
    ├─ YES: 경고, 재시도
    └─ NO: 계속 처리
    ↓

LDPC 디코딩 실패
    ├─ 최대 반복 도달 후 수렴 못함?
    ├─ YES: BER 증가, 패킷 손실 (통신 재시도)
    └─ NO: 정상 디코딩
    ↓

CRC 오류
    ├─ 계산값 ≠ 수신값?
    ├─ YES: 프레임 폐기, 재전송 요청
    └─ NO: 정상 수신
    ↓

PLL 불안정
    ├─ pll_freq 급변? (높은 도플러)
    ├─ YES: 루프 대역폭 증가 (PT_PLL_BANDWIDTH_SCALE ↑)
    └─ NO: 정상
    ↓

타이밍 불안정
    ├─ timing_error 진동?
    ├─ YES: 루프 대역폭 감소
    └─ NO: 정상
    ↓

메모리 부족
    ├─ malloc 실패?
    ├─ YES: 시스템 종료, 오류 로깅
    └─ NO: 계속
```

---

**완벽한 화살표 플로우 완성!** ✅
