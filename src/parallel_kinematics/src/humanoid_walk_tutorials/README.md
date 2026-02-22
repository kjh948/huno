  
 이 코드는 **Webots 시뮬레이터**에서 NAO 휴머노이드 로봇을 제어하는 **오픈루프 걸음걸이 컨트롤러**입니다. 주요 원리를 단계별로 설명드리겠습니다:

## 1. 기본 구조

```python
class Sprinter(Robot):
```
- `Robot` 클래스를 상속받아 NAO 로봇을 제어하는 `Sprinter` 클래스를 정의합니다.

## 2. 초기화 (`initialize` 메서드)

```python
def initialize(self):
    # 기본 시간 단계 설정 (시뮬레이션 타임스텝)
    self.timeStep = int(self.getBasicTimeStep())
    
    # 12개의 다리 모터 + 2개의 어깨 모터에 대한 포인터 획득
    self.RShoulderPitch = self.getDevice('RShoulderPitch')
    self.RHipYawPitch = self.getDevice('RHipYawPitch')
    # ... 등등
```

- 로봇의 관절 모터들에 대한 참조를 가져옵니다.
- NAO는 총 12개의 다리 관절(엉덩이 3개 × 2, 무릎 1개 × 2, 발목 2개 × 2)과 어깨 모터를 가집니다.

## 3. 핵심 원리: 병렬 운동학 (Parallel Kinematics)

```python
def left(self, x, y, z):
    # x, z 방향 (전후/상하)
    self.LKneePitch.setPosition(z)
    self.LHipPitch.setPosition(-z/2 + x)
    self.LAnklePitch.setPosition(-z/2 - x)
    
    # y 방향 (좌우)
    self.LHipRoll.setPosition(y)
    self.LAnkleRoll.setPosition(-y)
```

**핵심 가정**: 발이 항상 지면과 **평행**을 유지한다고 가정합니다.

이 가정 덕분에:
- **역운동학(Inverse Kinematics)** 계산을 단순화할 수 있음
- 무릎, 엉덩이, 발목의 각도를 간단한 수식으로 결정 가능

### 관절 각도 계산 원리:
- **무릎(`KneePitch`)**: 높이 `z`를 직접 반영
- **엉덩이(`HipPitch`)**: 무릎 각도의 절반을 보정 + 전후 위치 `x`
- **발목(`AnklePitch`)**: 무릎 각도의 절반을 반대로 보정 + 전후 위치 `x`
- **엉덩이/발목 롤(`Roll`)**: 좌우 위치 `y`를 반대 부호로 설정하여 발 평행 유지

## 4. 걸음걸이 생성 (`run` 메서드)

### 파라미터 설정:
```python
f = 4                    # 걸음 주파수 (속도 조절)
robot_height = 0.5       # 기본 몸체 높이
shift_y = 0.3            # 좌우 흔들림 폭
step_height = 0.4        # 발 들어올리는 높이
step_length = 0.2        # 보폭 (전후)
arm_swing = 2.0          # 팔 흔들림 크기
```

### 사인파 기반 리듬 생성:

```python
while self.step(self.timeStep) != -1:
    t = self.getTime() * f  # 시간 스케일링
    
    # 좌우 흔들림 (양발 동일)
    yLeftRight = math.sin(t) * shift_y
    
    # 발 높이 (번갈아 들어올림 - 위상차 π)
    zLeft = (math.sin(t) + 1.0) / 2.0 * step_height + robot_height
    zRight = (math.sin(t + math.pi) + 1.0) / 2.0 * step_height + robot_height
    
    # 보폭 (전후 이동 - 코사인파, 위상차 π)
    xLeft = math.cos(t) * step_length
    xRight = math.cos(t + math.pi) * step_length
```

### 운동 패턴 분석:

| 축 | 함수 | 의미 |
|---|---|---|
| **Y (좌우)** | `sin(t)` | 양발이 함께 좌우로 흔들림 (무게중심 이동) |
| **Z (상하)** | `(sin(t)+1)/2` | 0~1 범위로 변환하여 발을 번갈아 들어올림 |
| **X (전후)** | `cos(t)` | 보폭 생성 (왼발과 오른발 위상차 180°) |

**위상차 `math.pi`(180°)**: 한 발이 위로 올라갈 때 다른 발이 지면에 닿아 **교대 보행**을 만듭니다.

## 5. 팔 동작 (균형 유지)

```python
self.RShoulderPitch.setPosition(arm_swing * xLeft + math.pi/2 - 0.1)
self.LShoulderPitch.setPosition(arm_swing * xRight + math.pi/2 - 0.1)
```

- **반상보(反相補) 원리**: 왼발이 앞으로 나갈 때 오른팔이 앞으로, 오른발이 앞으로 나갈 때 왼팔이 앞으로 나갑니다.
- `math.pi/2 - 0.1`: 팔을 약간 앞으로 숙인 자세 (기본 자세)

## 6. 오픈루프 특성

이 컨트롤러는 **오픈루프(Open Loop)** 방식입니다:

- ✅ **장점**: 
  - 계산이 단순하고 빠름
  - 실시간 처리에 유리
  - 파라미터 튜닝으로 다양한 gait(걸음 패턴) 생성 가능

- ❌ **단점**:
  - 센서 피드백 없음 (균형 센서, 압력 센서 등 미사용)
  - 외란(밀림, 경사면 등)에 취약
  - 미끄러짐이나 넘어짐을 자동 복구하지 못함

## 7. 파라미터 세트 설명

코드에 주석 처리된 5가지 설정은 점진적으로 속도를 높여갑니다:

1. **Slow and steady** (f=4): 안정적인 저속 보행
2. **Walk it** (f=8): 일반적인 보행 속도
3. **Larger steps** (f=10): 보폭을 늘린 보행
4. **Tippel-tappel** (f=16): 빠른 발놀림 (작은 보폭)
5. **Run!!!** (f=14): 큰 보폭 + 높은 발들기 = 달리기

---

**요약**: 이 코드는 사인/코사인 파동을 이용해 주기적인 걸음걸이 패턴을 생성하고, "발이 항상 평행"이라는 가정 하에 간단한 기하학으로 관절 각도를 계산하는 **기본적인 오픈루프 보행 컨트롤러**입니다.


# Humanoid Walk Tutorials

Experiments and tutorials to create simple walking behavior on humanoid robots based on ideas of central pattern generators (CPG) and parallel kinematics (PK) assumptions.

## Simulation

For our experiments we use the Webots simulator:  
https://www.cyberbotics.com/

The aim of the tutorials is to solve the Humanoid Sprint Challenge:  
https://robotbenchmark.net/benchmark/humanoid_sprint/

The humanoid robot NAO:
https://cyberbotics.com/doc/guide/nao

## Approach

The humanoid Robot NAO has 25 joints (degrees of freedom), with 11 of them in its legs and the hip. The challenge is to control all the joints of the robot in a way that enables the robot to walk in a fast and stable manner. In the given scenario we focus on walking on a planar surface (no hills or slopes).

To simplify the task, we devise a simple *inverse kinematics* based on *parallel kinematics* assumptions. Instead of controlling a single joint, like a knee, we focus on controlling groups of joints that result in basic movements such as 
- contract a leg (z-axis)
- shift a foot forward (x-axis)
- shift a foot to the side (y-axis)
- turn a foot to a size (rotation on z-axis)
These basic movements allow us to construct the walk from the perspective of the feet, which is more intuitive.

To generate the walking pattern we use oscillations based on `sin`and `cos`. 
