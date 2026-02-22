# UVC Bipedal Walking Simulator

이 프로젝트는 Universal Virtual Control (UVC) 알고리즘을 기반으로 한 이족 보행 로봇 시뮬레이터입니다. C++로 작성된 원본 보행 엔진을 Python 및 PyBullet 환경으로 이식하여, Huno, Plen, GankenKun 등 다양한 로봇 모델에 대한 보행 시뮬레이션을 지원합니다.

## 1. UVC 구동 알고리즘 상세

UVC(상체 수직 제어) 알고리즘은 로봇의 기울기(IMU) 데이터를 피드백 받아 실시간으로 무게중심을 보정하는 직립 유지 알고리즘입니다.

### 1-1. 핵심 메커니즘
*   **상체 수직 제어 (Upper-body Vertical Control)**: 로봇의 몸체가 앞뒤(Pitch) 또는 좌우(Roll)로 기울어지면, 해당 방향의 지지 발 위치를 미세하게 변위시켜 상체가 항상 수직을 유지하도록 합니다.
*   **기본 보행 궤적 (Base Gait)**: 
    *   **Side Swing**: 양발 사이를 진동하며 체중 이동을 유도합니다 (Sine 파형).
    *   **Forward Swing**: 지지발은 뒤로 밀고, 스윙발은 앞으로 이동하며 전진합니다 (Linear/Cosine 보간).
*   **발 위치 IK (Inverse Kinematics)**: X(전진), Y(측면), H(높이) 좌표를 기반으로 고관절, 무릎, 발목의 5개 조인트 각도를 삼각함수를 통해 계산합니다.

### 1-2. 보행 상태 머신 (Gait States)
1.  **Mode 0 (Initialization)**: 로봇이 주저앉아 있는 상태에서 목표 보행 높이(`autoH`)까지 서서히 일어납니다.
2.  **Mode 10 (Idle)**: 보행 신호를 대기하며 정지 자세를 유지합니다.
3.  **Mode 20 (Double Support)**: 양발이 바닥에 닿은 상태에서 체중을 이동시키는 과도기적 구간입니다.
4.  **Mode 30 (Single Support)**: 한 발을 들어 올려 앞으로 내딛는 주 보행 구간입니다.

## 2. Plen 로봇 적용 상세

PLEN2 로봇 스케일에 맞추기 위해 다음과 같은 조정이 이루어졌습니다.

*   **스케일 최적화**: 원본 코드는 180mm 크기의 다리를 기준으로 하나, `plen.urdf`는 약 80mm 스케일입니다. 이를 반영하여 `LEG`, `autoH`, `swf` 등의 물리 파라미터를 소형 로봇에 맞춰 축소 적용했습니다.
*   **관절 매핑 (Joint Mapping)**: `lb_servo_l_hip`, `l_hip_l_thigh` 등 서보 위치 기반의 복잡한 URDF 조인트 명칭을 패턴 매칭을 통해 UVC 엔진의 논리적 조인트(Hip, Knee, Ankle)와 자동 연결합니다.
*   **회전 방향 보정 (Signs)**: 모델마다 다른 조인트 회전 축(Axis) 방향을 `signs` 설정을 통해 통합 관리합니다. Plen 모델은 모든 관절이 정방향(1)으로 설계된 특성을 반영했습니다.

## 3. 제어 파라미터 및 튜닝 가이드

시뮬레이션 안정화를 위해 `uvc.py`의 `configs` 섹션에서 다음 파라미터들을 조정할 수 있습니다.

| 파라미터 | 설명 | 튜닝 팁 |
| :--- | :--- | :--- |
| `leg_len` | 다리 전체 길이 (mm) | URDF의 Thigh + Shin 실측값과 일치해야 IK가 정확합니다. |
| `auto_h` | 기본 직립 높이 (mm) | 너무 낮으면 무릎이 너무 굽고, 너무 높으면 다리가 펴져서 보행 진동을 흡수하지 못합니다. |
| `swf` | 좌우 스윙 폭 (mm) | 로봇이 좌우로 너무 많이 흔들리면 값을 줄이고, 체중 이동이 안 되어 넘어지면 키웁니다. |
| `fwctEnd` | 한 걸음 프레임 수 | 보행 속도를 결정합니다. 값이 커질수록 느리고 신중하게 걷습니다. (기본 48) |
| `landRate` | 양발 지지 비율 | 양발이 바닥에 닿아 있는 시간 비중입니다. (기본 0.2) |
| `signs` | 조인트 회전 방향 계수 | 로봇이 걷지 않고 관절이 꼬인다면 해당 부위의 1/-1 값을 반전시킵니다. |

## 4. 실행 방법

```bash
# Plen 로봇 시뮬레이션
python uvc.py --robot plen

# Huno 로봇 시뮬레이션
python uvc.py --robot huno

# GUI 없이 백그라운드 테스트
python uvc.py --robot gankenkun --headless
```

---
*Created by [Your Name/Team] - 이 프로젝트는 교육용 보행 알고리즘 학습을 목적으로 제작되었습니다.*