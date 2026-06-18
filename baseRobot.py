from V5RPC import *
import math
import numpy as np
from GlobalVariable import *
import v5strategy

last_angle_dif = 0
last_dis_dif = 0

class PID:

    lastError = 0
    preError = 0
    now_error = 0
    lastU = 0

    def __init__(self, proportion, integral, derivative):
        self.proportion = proportion    # 比例常数
        self.integral = integral    # 积分常数
        self.derivative = derivative    # 微分常数

    def pid_cal(self, now_error):
        self.now_error = now_error
        now_pid = self.lastU + (self.proportion * (now_error - self.lastError) + self.integral * now_error
                   + self.derivative * (now_error - 2 * self.lastError + self.preError))

        return now_pid
    def update_pid(self):
        self.lastU += (self.proportion * (self.now_error - self.lastError) + self.integral * self.now_error
                       + self.derivative * (self.now_error - 2 * self.lastError + self.preError))
        self.preError = self.lastError
        self.lastError = self.now_error
    def reset_pid(self):
        self.lastError = 0
        self.preError = 0
        self.lastU = 0

# 老PID
class OLD_PID:
    def __init__(self, proportion, integral, derivative, lasterror, pre_error):
        self.proportion = proportion  # 比例常数
        self.integral = integral  # 积分常数
        self.derivative = derivative  # 微分常数
        self.lastError = lasterror  # 上一时刻的误差
        self.preError = pre_error  # 两个时刻前的误差


    def pid_cal(self, nowPoint):
        # 计算误差，这里是指角度误差
        # pid的最终目的是让机器人的运动朝向角度与所要跑位的位置角度差为0，也就是直勾勾跑到指定位置
        now_error = 0 - nowPoint

        # 计算当前次的输出
        """
        1.比例项：将当前误差减去上一次误差后，乘一个比例系数，来调节最终输出
        2.积分项：将当前误差乘以给定的积分系数，也就是误差积累来调节输出
        3.微分项：（当前误差 - 2*上一次误差 + 上上次误差）* 微分系数
        """
        now_pid = (self.proportion * (now_error - self.lastError) + self.integral * now_error
                   + self.derivative * (now_error - 2 * self.lastError + self.preError))

        # 更新误差
        self.preError = self.lastError
        self.lastError = now_error
        return now_pid

    def tune_parameters(self, oscillation_period):
        # 计算临界比例度
        Kcr = self.proportion
        # 根据系统类型选择参数整定规则
        self.proportion = 0.5 * Kcr
        self.integral = 1.2 / oscillation_period
        self.derivative = 0.075 * oscillation_period

class BaseRobot:
    def __init__(self):
        self.robot: Robot
        self.lastTargetX = 0
        self.lastTargetY = 0  # 机器人上一拍目标点
        self.lastRobotX = 0
        self.lastRobotY = 0   # 机器人上一排位置
        self.lastRotation = 0
        self.tick = GlobalVariable.tick

        self.lastU = 0  # pid控制变量U
        self.lastU1 = 0  # pid控制变量U1        

        self.HistoryInformation = [Robot(Vector2(0, 0), 0, Wheel(0, 0))] * 8
        self.PredictInformation = [Robot(Vector2(0, 0), 0, Wheel(0, 0))] * 8
        self.AngularSpeed = [0] * 8

        self.reset = False
        self.pid_oldMoveto = OLD_PID(0.25, 0, 3.8, 0, 0)
        # self.pid_moveto = PID(1.9, 0, 2.2)
        self.pid_moveto = PID(0.5,0,2)
        self.pid_moveto_dis = PID(2.48, 0, 6.06)
        self.pid_angle = PID(2, 0, 4.2)


    def update(self, env_robot: Robot, resetHistoryRecord):
        self.robot = env_robot
        self.reset = resetHistoryRecord
        '''
         # 代码黄方假设，但vda是蓝方假设，所以把所有坐标反转，旋转角也反转，从而可以是赋的轮速不变从而达到相同效果
        self.robot.position.x = -self.robot.position.x
        self.robot.position.y = -self.robot.position.y
        self.robot.rotation = self.robot.rotation - 180
        if self.robot.rotation < -180:
            self.robot.rotation += 360     
        '''

        if self.reset is True:
            for i in range(0, 8):
                self.HistoryInformation[i] = self.robot.copy()
        else:
            self.HistoryInformation[0] = self.robot.copy()



    def save_last_information(self, footBallNow_X, footBallNow_Y):	 # 保存机器人本拍信息，留作下一拍使用，可用于拓展保存更多信息
        self.lastRotation = self.robot.rotation
        self.lastRobotX = self.robot.position.x
        self.lastRobotY = self.robot.position.y
        self.lastBallx = footBallNow_X
        self.lastBally = footBallNow_Y

        # self.HistoryInformation.pop(0)

        for i in range(7, 0, -1):
            self.HistoryInformation[i] = self.HistoryInformation[i - 1].copy()
        self.HistoryInformation[1] = self.robot.copy()

        self.pid_moveto.update_pid()
        self.pid_angle.update_pid()
        return

    def get_pos(self):   # 获取机器人位置
        return self.robot.position

    def get_last_pos(self):	 # 获取机器人上一拍位置
        last_pos = Vector2(self.lastRobotX, self.lastRobotY)
        return last_pos

    def get_rotation(self):	 # 获取机器人旋转角
        return self.robot.rotation

    def get_left_wheel_velocity(self):	 # 获取机器人左轮速
        return self.robot.wheel.left_speed

    def get_right_wheel_velocity(self):	 # 获取机器人右轮速
        return self.robot.wheel.right_speed

    def set_wheel_velocity(self, vl, vr):	 # 直接赋左右轮速
        self.robot.wheel.left_speed = vl
        self.robot.wheel.right_speed = vr

    def moveto_dis (self, tar_x, tar_y):
        global last_angle_dif,last_dis_dif
        
        if self.tick == 1 or self.tick == 2 or self.tick == 3 or self.tick % 100 == 0 or self.reset is True: #比赛重启刚开始
            last_angle_dif = 0
            last_dis_dif = 0
            self.pid_moveto.reset_pid()
        v_max = 125

        dx = tar_x - self.PredictInformation[GlobalVariable.tick_delay].position.x
        dy = tar_y - self.PredictInformation[GlobalVariable.tick_delay].position.y
        angle_to = 180.0 / math.pi * np.arctan2(dy, dx)
        angle_diff = angle_to - self.PredictInformation[GlobalVariable.tick_delay].rotation
      
        dis_diff   = math.sqrt(dx * dx + dy * dy)
       
        'delta_v是转向换的速度'
        # if last_angle_dif != 0 :
        #     angle_diff = (angle_diff + last_angle_dif)/2
         
        while angle_diff > 180:
            angle_diff -= 360
        while angle_diff < -180:
            angle_diff += 360

        v_max = 0.3*dis_diff + 1.1*(dis_diff - last_dis_dif)
        # v_max =  22*dis_diff 
        if v_max >= 125 :
            v_max = 125
        if math.fabs(angle_diff) < 90:
            delta_v = self.pid_moveto.pid_cal(angle_diff)
            v_r = v_max + delta_v
            v_l = v_max - delta_v           
        elif math.fabs(angle_diff) >= 90:
            angle_diff += 180
            if angle_diff > 180:
                angle_diff -= 360
            delta_v = self.pid_moveto.pid_cal(angle_diff)
            v_r = -v_max + delta_v
            v_l = -v_max - delta_v
        # print ('v_max'+str(v_max),'delta_v'+str(delta_v))
        last_angle_dif =  angle_diff
        last_dis_dif = dis_diff

        self.set_wheel_velocity(v_l, v_r)
        
    def moveto(self, tar_x, tar_y):  # pid跑位函数
        if self.tick == 1 or self.tick == 2 or self.tick == 3 or self.tick % 100 == 0 or self.reset is True:
            self.pid_moveto.reset_pid()
        v_max = 125

        dx = tar_x - self.PredictInformation[GlobalVariable.tick_delay].position.x
        dy = tar_y - self.PredictInformation[GlobalVariable.tick_delay].position.y
        angle_to = 180.0 / math.pi * np.arctan2(dy, dx)
        angle_diff = angle_to - self.PredictInformation[GlobalVariable.tick_delay].rotation

        while angle_diff > 180:
            angle_diff -= 360
        while angle_diff < -180:
            angle_diff += 360
        if math.fabs(angle_diff) < 90:
            delta_v = self.pid_moveto.pid_cal(angle_diff)
            v_r = v_max + delta_v
            v_l = v_max - delta_v
        elif math.fabs(angle_diff) >= 90:
            angle_diff += 180
            if angle_diff > 180:
                angle_diff -= 360
            delta_v = self.pid_moveto.pid_cal(angle_diff)
            v_r = -v_max + delta_v
            v_l = -v_max - delta_v
        else:
            v_r = 100
            v_l = -100


        self.set_wheel_velocity(v_l, v_r)

    # pid跑位函数(老PID)
    def oldMoveto(self, tar_x, tar_y):
        v_max = 125
        dx = tar_x - self.get_pos().x
        dy = tar_y - self.get_pos().y
        angle_to = 180 / math.pi * np.arctan2(dy, dx)
        self.lastangle_to = angle_to
        angle_diff = self.get_rotation() - angle_to  # 计算误差角
        while angle_diff > 180:
            angle_diff -= 360
        while angle_diff < -180:
            angle_diff += 360
        if math.fabs(angle_diff) < 85:
            # 如果差角小于85，则正着走
            self.lastU = self.lastU + self.pid_oldMoveto.pid_cal(angle_diff)
            v_r = v_max + self.lastU  # 正行
            v_l = v_max - self.lastU
        elif math.fabs(angle_diff) >= 90:
            # 如果差角大于等于90，则倒着走
            # 这个90可以优化
            # 这两步计算正方向的补角，也就是准备倒车走
            angle_diff += 180
            if angle_diff > 180:
                angle_diff -= 360
            self.lastU = self.lastU + self.pid_oldMoveto.pid_cal(angle_diff)
            v_r = -v_max + self.lastU  # 倒着走
            v_l = -v_max - self.lastU
        else:
            # 在85到90之间，选择原地转圈
            # 这个80的轮速也可以自行调整
            v_r = 80
            v_l = -80
        self.set_wheel_velocity(v_l, v_r)

    def turntoangle(self, tar_angle):
        if self.tick == 1 or self.tick == 2 or self.tick == 3 or self.tick % 100 == 0 or self.reset is True:
            self.pid_moveto.reset_pid()
        v_max = 0
        angle_diff = tar_angle - self.PredictInformation[GlobalVariable.tick_delay].rotation

        while angle_diff > 180:
            angle_diff -= 360
        while angle_diff < -180:
            angle_diff += 360
        if math.fabs(angle_diff) < 90:        #
            delta_v = self.pid_angle.pid_cal(angle_diff)
            v_r = v_max + delta_v
            v_l = v_max - delta_v
        elif math.fabs(angle_diff) >= 90:  #背身跑过去
            angle_diff += 180
            if angle_diff > 180:
                angle_diff -= 360
            delta_v = self.pid_angle.pid_cal(angle_diff)
            v_r = -v_max + delta_v
            v_l = -v_max - delta_v
        else:
            v_r = 80
            v_l = -80
        self.set_wheel_velocity(v_l, v_r)

    def move_with_angle(self, tarx, tary, tar_angle):
        dx = tarx - self.PredictInformation[GlobalVariable.tick_delay].position.x
        dy = tary - self.PredictInformation[GlobalVariable.tick_delay].position.y
        distance = math.sqrt(dx * dx + dy * dy)

        if distance > 1:
            self.moveto(tarx, tary)
        else:
            self.turntoangle(tar_angle)

    def throw_ball(self, ballx, bally,dis):
        dx = ballx - self.PredictInformation[GlobalVariable.tick_delay].position.x
        dy = bally - self.PredictInformation[GlobalVariable.tick_delay].position.y
        distance = math.sqrt(dx * dx + dy * dy)

        if distance > dis or dx > 0: #离球距离大于10 或在右面则绕到左面
            self.move_with_angle(ballx + 1, bally, 180)
        else:
            if bally < 0:
                self.set_wheel_velocity(125, -125)
            else:
                self.set_wheel_velocity(-125, 125)
          
            

    def shoot(self, ballx, bally,dis): # 射门函数 球在-72.5右侧则追球进攻 -72.5左侧调用抛球函数射球

        if ballx > -72.5:
            self.moveto(ballx, bally)
        else:           #球门区  射门
            self.throw_ball(ballx, bally,dis)   
        

    def breakthrough(self, ballx,bally, tarx, tary):
        self.moveto(tarx, tary)
        future_time = 14
        len = 5
        deltay = 4
        
        futureOppy = future_time * bally - (future_time - 1) * ballx
        if (self.robot.position.x < ballx < tarx
                or self.robot.position.x > ballx > tarx):
            if self.robot.position.y > 0:
                if futureOppy + len > self.robot.position.y:
                    self.moveto(tarx, tary + deltay)
            if self.robot.position.y < 0:
                if futureOppy - len < self.robot.position.y:
                    self.moveto(tarx, tary - deltay)


    def moveto_within_x_limits(self, x_limit1, x_limit2, tar_x, tar_y):
        if self.get_pos().x < x_limit1:
            self.moveto(x_limit1, tar_y)
        else:
            if tar_x < x_limit1:
                self.moveto(x_limit1, tar_y)
            else:
                self.moveto(tar_x, tar_y)

        if self.get_pos().x > x_limit2:
            self.moveto(x_limit2, tar_y)
        else:
            if tar_x > x_limit2:
                self.moveto(x_limit2, tar_y)
            else:
                self.moveto(tar_x, tar_y)

    def move_in_still_x(self, still_x, football_y): #x不变
        self.moveto(still_x, football_y)

    def PredictRobotInformation(self, tick_delay):
        delta_t = 1
        self.PredictInformation[0] = self.robot.copy()

        self.AngularSpeed[0] = self.AngularSpeed[1]
        for i in range(1, tick_delay + 1):
            PredictedlastRotation = self.GetRobotInformation(i - 1).rotation
            PredictedLastPos = self.GetRobotInformation(i - 1).position.copy()
            # PredictedLineSpeed = (PredictedLastPos - self.GetRobotInformation(i - 2).position) / delta_t # i-1拍线速度
            #PredictedLineSpeed = math.sqrt(pow(PredictedLastPos.x - self.GetRobotInformation(i - 2).position.x, 2) + pow(PredictedLastPos.y - self.GetRobotInformation(i - 2).position.y, 2))

            dx = PredictedLastPos.x - self.GetRobotInformation(i - 2).position.x
            dy = PredictedLastPos.y - self.GetRobotInformation(i - 2).position.y

            PredictedLineSpeed = math.sqrt(dx * dx + dy * dy)
            
            angle_logic = math.atan2(dy, dx) * 180 / math.pi	#逻辑角度
            if PredictedLastPos.x < self.GetRobotInformation(i - 2).position.x:
                if angle_logic > 0:
                    angle_logic = 180 - angle_logic
                else:
                    angle_logic = -180 - angle_logic
            delta_angle = angle_logic - self.GetRobotInformation(i - 1).rotation
            if delta_angle > 180:
                delta_angle -= 360
            if delta_angle < -180:
                delta_angle += 360
            if math.fabs(delta_angle) <= 90:  #基于蓝方
                PredictedLineSpeed = -PredictedLineSpeed
                
            if (self.AngularSpeed[i - 1] > 0):
                if PredictedlastRotation > self.GetRobotInformation(i - 2).rotation:
                    PiedictedAngularSpeed = PredictedlastRotation - self.GetRobotInformation(i - 2).rotation
                else:
                    PiedictedAngularSpeed = (360 + PredictedlastRotation - self.GetRobotInformation(i - 2).rotation)
            else:
                if PredictedlastRotation > self.GetRobotInformation(i - 2).rotation:
                    PiedictedAngularSpeed = PredictedlastRotation - self.GetRobotInformation(i - 2).rotation - 360
                else:
                    PiedictedAngularSpeed = (PredictedlastRotation - self.GetRobotInformation(i - 2).rotation)

            settedWheelSpeed = self.HistoryInformation[tick_delay + 1 - i].wheel.copy()
            if settedWheelSpeed.left_speed > 125:
                settedWheelSpeed.left_speed = 125
            elif settedWheelSpeed.left_speed < -125:
                settedWheelSpeed.left_speed = -125
            if settedWheelSpeed.right_speed > 125:
                settedWheelSpeed.right_speed = 125
            elif settedWheelSpeed.right_speed < -125:
                settedWheelSpeed.right_speed = -125

            K1 = 0.002362192
            K2 = math.exp(-1 / 0.9231)
            K3 = 1 - math.exp(-1 / 3.096)
            K4 = 0.53461992

            if settedWheelSpeed.left_speed != 0 and settedWheelSpeed.right_speed != 0:
                nextLineSpeed = PredictedLineSpeed * 0.939534127623834133 + (settedWheelSpeed.left_speed + settedWheelSpeed.right_speed) / 2 * K1
            if settedWheelSpeed.left_speed == 0 or settedWheelSpeed.right_speed == 0:
                nextLineSpeed = PredictedLineSpeed * K2

            nextAngularSpeed = PiedictedAngularSpeed + ((settedWheelSpeed.right_speed - settedWheelSpeed.left_speed) / 2 * K4 - PiedictedAngularSpeed) * K3
            self.AngularSpeed[i] = nextAngularSpeed

            newLineSpeed = nextLineSpeed    # 先算线速度然后根据线速度算出位置？
            newAngularSpeed = nextAngularSpeed  # 先算角速度然后根据角速度算出角度？（假设角速度ni时针为正）

            self.PredictInformation[i].position.x = self.PredictInformation[i - 1].position.x + newLineSpeed * math.cos(self.PredictInformation[i-1].rotation / 180 * math.pi)
            self.PredictInformation[i].position.y = self.PredictInformation[i - 1].position.y + newLineSpeed * math.sin(self.PredictInformation[i-1].rotation / 180 * math.pi)
            self.PredictInformation[i].rotation = self.PredictInformation[i - 1].rotation + newAngularSpeed

            #print(newLineSpeed)
            #print(math.cos(self.PredictInformation[i-1].rotation / 180 * math.pi))
            #print(self.PredictInformation[i].position.y)
            while self.PredictInformation[i].rotation > 180:
                self.PredictInformation[i].rotation -= 360
            while self.PredictInformation[i].rotation < -180:
                self.PredictInformation[i].rotation += 360


    def GetRobotInformation(self, time):
        if time >= 0:
            return self.PredictInformation[time]
        else:
            return self.HistoryInformation[-time]

class DataLoader:
    def get_event(self, tick):  # 获得tick时刻的比赛状态
        return self.event_states[tick]

    def set_tick_state(self, tick, event_state):    # 设置此时的信息
        self.tick = tick
        self.event_states[tick] = event_state
    tick = 0
    event_states = [-1 for n in range(100000)]
