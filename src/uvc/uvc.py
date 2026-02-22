import numpy as np
import pybullet as p
import pybullet_data
import time
import argparse
import os
import json

class JointMapping:
    """
    URDF 조인트 이름과 PyBullet 인덱스 간의 매핑을 관리하는 클래스.
    로봇마다 다른 조인트 명칭을 유연하게 처리하기 위해 패턴 매칭 방식을 사용합니다.
    """
    def __init__(self, robot_id):
        self.robot_id = robot_id
        self.joint_name_to_id = {}
        self.num_joints = p.getNumJoints(robot_id)
        # 로봇의 모든 조인트 정보를 읽어 이름:ID 맵 생성
        for i in range(self.num_joints):
            info = p.getJointInfo(robot_id, i)
            name = info[1].decode('utf-8')
            self.joint_name_to_id[name] = i
            
    def get_id(self, name_patterns):
        """
        주어진 패턴 목록 중 하나라도 포함된 조인트 이름을 찾아 ID를 반환합니다.
        """
        if isinstance(name_patterns, str):
            name_patterns = [name_patterns]
            
        for name in self.joint_name_to_id:
            for pattern in name_patterns:
                if pattern in name:
                    return self.joint_name_to_id[name]
        return None

class UVCCore:
    """
    Universal Virtual Control (UVC) 기반 보행 패턴 생성 엔진.
    C++ 원본 로직을 Python으로 이식하였으며, mm 단위를 m 단위로 스케일링하여 처리합니다.
    """
    def __init__(self, scale=0.001):
        # 단위 변환 계수 (주로 mm -> m)
        self.scale = scale
        self.adjFR = 2.04 * self.scale  # 앞뒤 정렬 보정값
        self.autoH = 170.0 * self.scale # 목표 보행 높이
        self.autoHs = 180.0 * self.scale # 초기 서기 높이
        self.LEG = 180.0 * self.scale    # 다리 전체 길이 (Thigh + Shin)
        
        # 상태 제어 변수
        self.mode = 0      # 0:초기화, 10:대기, 20/30:보행
        self.walkF = 0     # 보행 시작 플래그
        self.jikuasi = 0   # 지지각 (0:오른발, 1:왼발)
        self.fwct = 0      # 보행 주기 카운터 (Frame Counter)
        self.fwctEnd = 48  # 한 걸음 당 총 프레임 수
        
        # 보행 파라미터 (Displacement)
        self.dx = [0.0, 0.0] # 각 발의 앞뒤 이동량
        self.fwr0 = 0.0      # 이전 걸음의 종점 위치
        self.fwr1 = 0.0      # 이전 걸음의 시작 위치
        self.dxi = 0.0       # UVC에 의한 앞뒤 보정량
        self.dyi = 0.0       # UVC에 의한 좌우 보정량
        self.dy = 0.0        # 기본 좌우 스윙량
        self.swf = 12.0 * self.scale # 최대 좌우 스윙 폭
        self.fhMax = 20.0 * self.scale # 발 들어올리기 최대 높이
        self.landRate = 0.2  # 양발 지지 구간 비율
        self.fh = 0.0        # 현재 프레임의 발 높이
        self.fw = 0.0        # 보폭 (Step length)
        
        # 조인트 목표 각도 (radians)
        # s=0: Right, s=1: Left
        self.K0W = [0.0, 0.0]  # Hip Pitch (고관절 앞뒤)
        self.K1W = [0.0, 0.0]  # Hip Roll  (고관절 좌우)
        self.HW = [0.0, 0.0]   # Knee Pitch (무릎)
        self.A0W = [0.0, 0.0]  # Ankle Pitch (발목 앞뒤)
        self.A1W = [0.0, 0.0]  # Ankle Roll  (발목 좌우)
        
        # 센서 피드백 변수
        self.uvcOff = 0       # UVC 활성화 여부
        self.asiPress_r = 0.0 # 오른발 압력 (z축 힘)
        self.asiPress_l = 0.0 # 왼발 압력
        self.fbRad = 0.0      # 앞뒤 기울기 (Pitch)
        self.lrRad = 0.0      # 좌우 기울기 (Roll)
        self.fbAV = 0.0       # 앞뒤 각속도 (Gyro)
        self.lrAV = 0.0       # 좌우 각속도

    def foot_cont(self, x, y, h, s):
        """
        역기구학(IK) 계산: 주어진 발 위치(x, y)와 높이(h)를 조인트 각도로 변환.
        """
        # 피타고라스 정리를 이용한 다리 직선 길이 k 계산
        k = np.sqrt(x*x + (y*y + h*h))
        if k > self.LEG: k = self.LEG # 최대 길이 제한
            
        # 1. Pitch 방향 IK (고관절, 무릎, 발목)
        # 사인/코사인 법칙을 이용한 각도 산출
        x_angle = np.arcsin(x/k)
        k_angle = np.arccos(k/self.LEG)
        
        self.K0W[s] = k_angle + x_angle # Hip Pitch
        self.HW[s] = k_angle * 2.0      # Knee Pitch (양 다리 길이가 같으므로 2배)
        self.A0W[s] = k_angle - x_angle - 0.003 * self.fbAV # Ankle Pitch (자이로 보정 포함)
        
        # 2. Roll 방향 IK (고관절, 발목)
        roll_angle = np.arctan(y/h)
        self.K1W[s] = roll_angle # Hip Roll
        
        # 발목 수평 유지를 위한 Roll 제어
        if s == 0: # 오른발
            self.A1W[s] = -roll_angle - 0.002 * self.lrAV
        else: # 왼발
            self.A1W[s] = -roll_angle + 0.002 * self.lrAV

    def step(self):
        """
        보행 엔진의 메인 루프 프레임 업데이트.
        상태 머신을 통해 준비 -> 보행 순으로 진행됩니다.
        """
        if self.mode == 0: # 초기 자세(Standing up)
            if self.autoHs > self.autoH:
                self.autoHs -= 1.0 * self.scale
            else:
                self.mode = 10 # 대기 상태로 전환
            self.foot_cont(-self.adjFR, 0, self.autoHs, 0)
            self.foot_cont(-self.adjFR, 0, self.autoHs, 1)
            
        elif self.mode == 10: # 대기(Idle/Ready)
            self.K0W = [0.0, 0.0]
            self.dx, self.fwr0, self.fwr1, self.fwct = [0, 0], 0, 0, 0
            self.dxi, self.dyi, self.dy, self.jikuasi = 0, 0, 0, 0
            
            self.foot_cont(-self.adjFR, 0, self.autoH, 0)
            self.foot_cont(-self.adjFR, 0, self.autoH, 1)
            
            if (self.walkF & 0x01): # 보행 시작 신호 확인
                self.fw = 20.0 * self.scale # 첫 보폭 설정
                self.mode = 20
                
        elif self.mode in [20, 30]: # 보행 중 (Walking)
            # --- 1. UVC (Upper-body Vertical Control) 엔진 ---
            # 기울기에 따라 무게중심 이동을 보정하여 직립 유지
            if (self.jikuasi == 0 and self.asiPress_r < -0.1 and self.asiPress_l > -0.1) or \
               (self.jikuasi == 1 and self.asiPress_r > -0.1 and self.asiPress_l < -0.1):
                
                # Roll 피드백을 통한 좌우 변위 보정
                k_lr = 1.5 * 193 * self.scale * np.sin(self.lrRad)
                if self.jikuasi == 0: self.dyi += k_lr
                else: self.dyi -= k_lr
                
                self.dyi = np.clip(self.dyi, -30 * self.scale, 0)
                # Pitch 피드백을 통한 앞뒤 변위 보정
                self.dxi += (1.5 * 130 * self.scale * np.sin(self.fbRad))
                
            self.dyi *= 0.90 # 보정값 감쇠 (LDF 느낌의 필터)
            if self.uvcOff == 1: self.dxi, self.dyi = 0, 0
                
            # --- 2. 기본 보행 궤적(Base Gait) 생성 ---
            # 사인파 기반의 좌우 진동(Side Swing)
            k_sin = np.sin(np.pi * self.fwct / self.fwctEnd)
            self.dy = (k_sin * self.swf) if self.jikuasi == 0 else (-k_sin * self.swf)
                
            # 지지발의 앞뒤 이동 (Support Foot Reverse Swing)
            if self.fwct < self.fwctEnd / 2:
                self.dx[self.jikuasi] = self.fwr0 * (1.0 - 2.0 * self.fwct / self.fwctEnd)
            else:
                self.dx[self.jikuasi] = -(self.fw - self.dxi) * (2.0 * self.fwct / self.fwctEnd - 1.0)
                
            # 스윙발의 앞뒤 이동 (Swing Foot Forward Swing)
            if self.mode == 20: # 초기 체중 이동 구간 (Both foot support)
                if self.fwct < (self.landRate * self.fwctEnd):
                    self.dx[self.jikuasi ^ 1] = self.fwr1 - (self.fwr0 - self.dx[self.jikuasi])
                else:
                    self.fwr1 = self.dx[self.jikuasi ^ 1]
                    self.mode = 30 # 완전 스윙 구간으로 전환
            
            if self.mode == 30: # 궤적 보정 구간 (Swing phase)
                prog = (self.fwct - self.landRate * self.fwctEnd) / ((1.0 - self.landRate) * self.fwctEnd)
                k_int = (-np.cos(np.pi * prog) + 1.0) / 2.0 # Cosine interpolation
                self.dx[self.jikuasi ^ 1] = self.fwr1 + k_int * (self.fw - self.dxi - self.fwr1)
                
            # 이동 한계값 제한
            limit = 100 * self.scale
            self.dx[0] = np.clip(self.dx[0], -limit, limit)
            self.dx[1] = np.clip(self.dx[1], -limit, limit)
            
            # 발 들어올리기 높이 계산 (Swing foot lift)
            i_start = self.landRate * self.fwctEnd
            self.fh = (self.fhMax * np.sin(np.pi * (self.fwct - i_start) / (self.fwctEnd - i_start))) if self.fwct > i_start else 0
            
            # --- 3. 최종 조인트 제어 명령 생성 ---
            if self.jikuasi == 0: # 오른발 지지
                self.foot_cont(self.dx[0] - self.adjFR, -self.dy - self.dyi + self.scale, self.autoH, 0)
                self.foot_cont(self.dx[1] - self.adjFR,  self.dy - self.dyi + self.scale, self.autoH - self.fh, 1)
            else: # 왼발 지지
                self.foot_cont(self.dx[0] - self.adjFR, -self.dy - self.dyi + self.scale, self.autoH - self.fh, 0)
                self.foot_cont(self.dx[1] - self.adjFR,  self.dy - self.dyi + self.scale, self.autoH, 1)
                
            # --- 4. 한 걸음 종료 및 지지각 교체 ---
            if self.fwct == self.fwctEnd:
                self.jikuasi ^= 1
                self.fwct, self.dxi, self.fh, self.mode = 1, 0, 0, 20
                self.fwr0, self.fwr1 = self.dx[self.jikuasi], self.dx[self.jikuasi ^ 1]
                if self.fw == 20 * self.scale: # 처음 시작시 보폭 조절
                    self.landRate, self.fw = 0.1, 40 * self.scale
            else:
                self.fwct += 1

class BipedSim:
    """
    PyBullet 물리 시뮬레이션 환경을 총괄하는 클래스.
    환경 설정, 로봇 로딩, 센서 데이터 획득 및 조인트 제어를 담당합니다.
    """
    def __init__(self, urdf_path, headless=False):
        self.urdf_path = urdf_path
        self.mode = p.DIRECT if headless else p.GUI # Headless 지원
        self.physics_client = p.connect(self.mode)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81) # 표준 중력 가속도
        
        self.plane_id = p.loadURDF("plane.urdf") # 지면(평면) 생성
        # 초기 높이를 지면보다 약간 높게 설정하여 로드 (바닥 보정용)
        self.robot_id = p.loadURDF(urdf_path, [0, 0, 0.5], useFixedBase=False)
        
        # 로봇의 위치를 지면에 딱 맞게 조정 (AABB 기반)
        self._adjust_initial_height()
        
        self.mapper = JointMapping(self.robot_id)
        self.ids = self._auto_map_joints()
        
    def _adjust_initial_height(self):
        """
        로봇의 모든 링크 중 가장 낮은 지점(발바닥)이 Z=0에 오도록 위치를 재설정합니다.
        """
        min_z = 9999
        for i in range(-1, p.getNumJoints(self.robot_id)):
            aabb = p.getAABB(self.robot_id, i)
            if aabb[0][2] < min_z:
                min_z = aabb[0][2]
        
        pos, orn = p.getBasePositionAndOrientation(self.robot_id)
        new_pos = [pos[0], pos[1], pos[2] - min_z]
        p.resetBasePositionAndOrientation(self.robot_id, new_pos, orn)

    def _auto_map_joints(self):
        """
        URDF의 문자열 조인트 이름을 힌트를 통해 해당 조인트 ID로 매핑합니다.
        Huno, Plen, GankenKun 등 다양한 로봇을 동시에 지원합니다.
        """
        patterns = {
            'l_hip_pitch': ['l_hip_pitch', 'left_waist_pitch', 'left_thigh_pitch', 'leg_l_thigh_pitch', 'l_hip_l_thigh'],
            'l_hip_roll': ['l_hip_roll', 'left_waist_roll', 'left_thigh_roll', 'leg_l_thigh_roll', 'lb_servo_l_hip'],
            'l_knee': ['l_knee_pitch', 'left_knee_pitch', 'left_knee', 'leg_l_knee_pitch', 'l_knee_l_shin'],
            'l_ankle_pitch': ['l_ankle_pitch', 'left_ankle_pitch', 'left_foot_pitch', 'leg_l_ankle_pitch', 'l_shin_l_ankle'],
            'l_ankle_roll': ['l_ankle_roll', 'left_ankle_roll', 'left_foot_roll', 'leg_l_ankle_roll', 'l_ankle_l_foot'],
            'r_hip_pitch': ['r_hip_pitch', 'right_waist_pitch', 'right_thigh_pitch', 'leg_r_thigh_pitch', 'r_hip_r_thigh'],
            'r_hip_roll': ['r_hip_roll', 'right_waist_roll', 'right_thigh_roll', 'leg_r_thigh_roll', 'rb_servo_r_hip'],
            'r_knee': ['r_knee_pitch', 'right_knee_pitch', 'right_knee', 'leg_r_knee_pitch', 'r_knee_r_shin'],
            'r_ankle_pitch': ['r_ankle_pitch', 'right_ankle_pitch', 'right_foot_pitch', 'leg_r_ankle_pitch', 'r_shin_r_ankle'],
            'r_ankle_roll': ['r_ankle_roll', 'right_ankle_roll', 'right_foot_roll', 'leg_r_ankle_roll', 'r_ankle_r_foot'],
        }
        return {key: self.mapper.get_id(pats) for key, pats in patterns.items()}

    def get_sensors(self):
        """
        IMU(오리엔테이션, 각속도) 및 지면 압력 센서 데이터를 획득합니다.
        """
        # 1. IMU 데이터 (Base Link)
        pos, orn = p.getBasePositionAndOrientation(self.robot_id)
        lin_vel, ang_vel = p.getBaseVelocity(self.robot_id)
        euler = p.getEulerFromQuaternion(orn)
        
        # 월드 좌표 각속도를 로컬 좌표계로 변환 (자이로 데이터 모사)
        rot_matrix = np.array(p.getMatrixFromQuaternion(orn)).reshape(3, 3)
        local_ang_vel = rot_matrix.T @ np.array(ang_vel)
        
        # 2. 압력 센서 데이터 (발바닥 접촉 힘)
        contacts = p.getContactPoints(bodyA=self.robot_id)
        press_r, press_l = 0.0, 0.0
        for c in contacts:
            force = c[9] # Normal force
            link_index = c[3]
            if link_index == -1:
                link_name = "base_link"
            else:
                link_name = p.getJointInfo(self.robot_id, link_index)[12].decode('utf-8')
            
            # 발 부위의 접촉 힘을 합산
            if 'l_foot' in link_name: press_l -= force
            elif 'r_foot' in link_name: press_r -= force
            
        return {
            'roll': euler[0], 'pitch': euler[1],
            'roll_av': local_ang_vel[0], 'pitch_av': local_ang_vel[1],
            'press_r': press_r / 100.0, 'press_l': press_l / 100.0
        }

    def apply_control(self, K0W, K1W, HW, A0W, A1W, signs):
        """
        생성된 목표 각도를 로봇의 실제 조인트에 명령으로 전달합니다.
        로봇별 회전 방향(signs)을 고려하여 적용합니다.
        """
        # 모델별 정/역방향 계수 가져오기
        hip_p = signs.get('hip_p', 1)
        hip_r = signs.get('hip_r', 1)
        knee = signs.get('knee', 1)
        ankle_p = signs.get('ankle_p', 1)
        ankle_r = signs.get('ankle_r', 1)

        targets = {
            'l_hip_pitch': hip_p * K0W[1], 
            'l_hip_roll': hip_r * K1W[1], 
            'l_knee': knee * HW[1], 
            'l_ankle_pitch': ankle_p * A0W[1], 
            'l_ankle_roll': ankle_r * A1W[1],
            
            'r_hip_pitch': hip_p * K0W[0], 
            'r_hip_roll': hip_r * K1W[0], 
            'r_knee': knee * HW[0], 
            'r_ankle_pitch': ankle_p * A0W[0], 
            'r_ankle_roll': ankle_r * A1W[0]
        }
        for name, target in targets.items():
            jid = self.ids.get(name)
            if jid is not None:
                # POSITION_CONTROL (서보 모터 방식)으로 제어
                p.setJointMotorControl2(self.robot_id, jid, p.POSITION_CONTROL, targetPosition=target)

def main():
    parser = argparse.ArgumentParser(description="UVC Bipedal Walking Simulator")
    parser.add_argument('--robot', type=str, default='plen', choices=['huno', 'plen', 'gankenkun'], help='Robot type')
    parser.add_argument('--headless', action='store_true', help='Run simulation without GUI')
    args = parser.parse_args()
    
    # 로봇별 상세 물리 및 제어 설정
    configs = {
        'huno': {
            'urdf': '../huno_base/src/urdf/huno.urdf',
            'leg_len': 180.0, # mm 기준
            'auto_h': 170.0,
            'swf': 12.0,      # 좌우 요동 폭
            'signs': {'hip_p': -1, 'hip_r': 1, 'knee': -1, 'ankle_p': -1, 'ankle_r': 1}
        },
        'plen': {
            'urdf': 'urdf/plen.urdf',
            'leg_len': 80.0,  # 소형 로봇 스케일
            'auto_h': 75.0,
            'swf': 8.0,
            'signs': {'hip_p': 1, 'hip_r': 1, 'knee': 1, 'ankle_p': 1, 'ankle_r': 1}
        },
        'gankenkun': {
            'urdf': '../huno_base/src/GankenKun_pybullet/URDF/gankenkun.urdf',
            'leg_len': 200.0,
            'semi_leg': 100.0,
            'auto_h': 185.0,
            'swf': 20.0,
            'signs': {'hip_p': -1, 'hip_r': 1, 'knee': -1, 'ankle_p': -1, 'ankle_r': 1}
        }
    }
    
    config = configs.get(args.robot, configs['huno'])
    
    # 1. URDF 파일 경로 구성
    script_dir = os.path.dirname(__file__)
    urdf = os.path.abspath(os.path.join(script_dir, config['urdf']))
    
    if not os.path.exists(urdf):
        print(f"Warning: URDF not found at {urdf}, falling back to huno.urdf")
        urdf = os.path.abspath(os.path.join(script_dir, configs['huno']['urdf']))
        config = configs['huno']
        
    # 2. 시뮬레이션 및 코어 엔진 초기화
    sim = BipedSim(urdf, headless=args.headless)
    core = UVCCore(scale=0.001)
    
    # 로봇별 파라미터 코어 엔진에 주입
    core.LEG = config['leg_len'] * core.scale
    if 'auto_h' in config:
        core.autoH = config['auto_h'] * core.scale
        core.autoHs = (config['auto_h'] + 10.0) * core.scale
    core.swf = config['swf'] * core.scale
    
    core.walkF = 0x01 # 보행 플래그 ON
    
    # 3. 시뮬레이션 타임스텝 설정 (0.01초 = 100Hz 제어)
    dt = 0.01
    p.setTimeStep(dt)
    
    print(f"Starting UVC walking simulation for [{args.robot}]...")
    try:
        while True:
            # --- 센서 단계 ---
            sensors = sim.get_sensors()
            core.lrRad, core.fbRad = sensors['roll'], sensors['pitch']
            core.lrAV, core.fbAV = sensors['roll_av'], sensors['pitch_av']
            core.asiPress_r, core.asiPress_l = sensors['press_r'], sensors['press_l']
            
            # --- 제어 알고리즘 단계 ---
            core.step()
            
            # --- 실뮬레이션 적용 단계 ---
            sim.apply_control(core.K0W, core.K1W, core.HW, core.A0W, core.A1W, config['signs'])
            
            p.stepSimulation() # 물리 엔진 1스텝 전진
            if not args.headless: time.sleep(dt) # 실시간 속도 맞춤
    except KeyboardInterrupt:
        print("\nSimulation stopped by user.")
    finally:
        p.disconnect()

if __name__ == '__main__':
    main()
