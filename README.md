# Missile-Pursuit-Simulation

Missile Pursuit Simulation (사미용두B 팀)
본 레포지토리는 MATLAB 및 Simulink 환경에서 구현된 고도화된 2D(BEV) 대공 요격 시뮬레이션입니다.
살기 위해 지그재그로 회피 기동을 펼치는 동적 표적을 상대로, 단순한 추적이 아닌 미래의 예상 교차점을 계산하여 타격하는 비례항법(PNG, TPN/APN) 유도 법칙을 핵심 알고리즘으로 사용합니다.

주요 구현 사항:

Guidance: 타겟 가속도를 보상하는 확장 비례항법(APN) 알고리즘

Autopilot: 제어 지연과 G-force 한계를 고려한 현실적인 제어기

Kinematics: 3자유도(3DOF) 물리 엔진을 통한 실시간 궤적 산출

Avoidance: VFH(Vector Field Histogram) 기반 전방 장애물 회피

## ⚙️ Core Pipeline
`가상 센서 인지부` ➡️ `모드 스위칭 (VFH / APN / TPN)` ➡️ `측방향 가속도 연산` ➡️ `Autopilot (PID 제어)` ➡️ `3DOF 물리 엔진` ➡️ `화면 시각화 및 피드백`
