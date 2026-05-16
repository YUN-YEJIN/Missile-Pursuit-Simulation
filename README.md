# Missile-Pursuit-Simulation

Missile Pursuit Simulation (사미용두B 팀)
본 레포지토리는 MATLAB 및 Simulink 환경에서 구현된 PN 기반 고도화된 2D 미사일 요격 시뮬레이션입니다.
추격되지 않기 위해 지그재그로 회피 기동을 펼치는 동적 표적을 상대로, 단순한 추적이 아닌 미래의 예상 교차점을 계산하여 타격하는 비례항법(PNG, TPN/APN) 유도 법칙을 핵심 알고리즘으로 사용합니다.

# ⚙️ Core Pipeline
`가상 센서 인지부` ➡️ `모드 스위칭 (VFH / APN / TPN)` ➡️ `측방향 가속도 연산` ➡️ `Autopilot (PID 제어)` ➡️ `3DOF 물리 엔진` ➡️ `화면 시각화 및 피드백`

# Only_PNG_Example 폴더
설명: 초기 단계로 "PNG 기법"만을 사용하여 interceptor(요격체)가 target(목표물)을 격추하는 matlab 파일이 들어있음

## png_targetmode_plot 파일
- PNG(비례항법유도) 기법을 사용하여 Chaser(추종체)가 Target(목표물)을 추격하는 MATLAB 시뮬레이션 파일.  
- Target의 행동 모드를 5가지 중 선택할 수 있으며 실시간 애니메이션으로 궤적을 확인할 수 있음.
---

### 1. 초기 조건 입력
- 처음 실행하면 Target의 초기 상태를 직접 입력해야 합니다.
  
Target initial x position [m] =

Target initial y position [m] = 

Target speed [m/s] = 

Target heading angle [deg] =

---

### 2. mode 선택

이후 숫자 입력으로 Target 행동 모드를 고를 수 있습니다.

| 입력값 | 모드 | 설명 |
|--------|------|------|
| `1` | 직진 | 방향 변화 없이 직진 |
| `2` | 회피형 | 좌우로 흔들리며 회피 |
| `3` | 랜덤형 | 일정 주기마다 랜덤 방향 전환 |
| `4` | 반응형 | Chaser 감지 시 반대 방향으로 회피 |
| `5` | 패닉 회피형 | Chaser 접근 시 급격한 회피 |
| `0` | 랜덤 전환 | 3초마다 1~5 중 랜덤 전환 |

---

### 3. 회피형 (mode 2)

좌우로 일정 주기마다 방향을 바꾸며 흔들리는 형태.  -> **"단순 회피 기동"**

```matlab
targetConfig.evasive_turn_rate_deg = 18;   % 회전 속도 [deg/s]
targetConfig.evasive_flip_interval = 1.2;  % 방향 전환 주기 [s]
```

---

### 4. 랜덤형 (mode 3)

일정 시간마다 랜덤한 회전율을 뽑아서 움직입니다.  
값을 키우면 더 난폭하게 움직입니다.

```matlab
targetConfig.random_turn_rate_max_deg = 30;  % 최대 회전율 [deg/s]
targetConfig.random_update_interval = 1.0;   % 갱신 주기 [s]
```

---

### 5. 반응형 (mode 4)

Chaser가 일정 거리 안으로 들어오면 반대 방향으로 도는 형태.

```matlab
targetConfig.reactive_turn_rate_deg = 28;  % 회피 회전 속도 [deg/s]
targetConfig.reactive_range = 700;         % 반응 거리 [m]
```

---

### 6. 패닉 회피형 (mode 5)

Chaser가 가까워지면 급격하게 회피하는 형태.  
회피 후 일정 거리 이상 멀어져야 패닉 상태가 해제됩니다.

```matlab
targetConfig.panic_range = 700;          % 패닉 진입 거리 [m]
targetConfig.recover_range = 850;        % 패닉 해제 거리 [m]
targetConfig.panic_turn_rate_deg = 85;   % 패닉 회전 속도 [deg/s]
targetConfig.panic_hold_time = 1.0;      % 최소 패닉 유지 시간 [s]
```

---

### 7. 주요 파라미터

```matlab
C_speed = 30;           % Chaser 속도 [m/s]
N = 3;                  % 비례항법 상수
a_max = 120;            % 최대 횡방향 가속도 [m/s²]
encounter_radius = 10;  % 요격 판정 거리 [m]
```
