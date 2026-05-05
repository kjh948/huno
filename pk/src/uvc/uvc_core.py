import numpy as np

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
