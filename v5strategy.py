'''
这个文件是主要开发文件，涵盖了策略全部的四个接口
-on_event接收比赛状态变化的信息。
    参数event_type type表示事件类型；
    参数EventArgument表示该事件的参数，如果不含参数，则为NULL。
-get_team_info控制队名。
    修改返回值的字符串即可修改自己的队名
-get_instruction控制5个机器人的轮速(leftspeed,rightspeed)，以及最后的reset(1即表明需要reset)
    通过返回值来给机器人赋轮速
    比赛中的每拍被调用，需要策略指定轮速，相当于旧接口的Strategy。
    参数field为In/Out参数，存储当前赛场信息，并允许策略修改己方轮速。
    ！！！所有策略的开发应该在此模块
-get_placement控制5个机器人及球在需要摆位时的位置
    通过返回值来控制机器人和球的摆位。
    每次自动摆位时被调用，需要策略指定摆位信息。
    定位球类的摆位需要符合规则，否则会被重摆
'''
import random
from typing import Tuple, Union

import numpy as np
from V5RPC import *
import math
from baseRobot import *
from GlobalVariable import *

baseRobots = []# 定义我方机器人数组
oppRobots = []# 定义对方机器人数组
data_loader = DataLoader()
race_state = -1  # 定位球状态
race_state_trigger = -1    # 触发方
num_goalkepper = -1         #对方守门员编号 1代表有守门员  其余为有
penalty = 1               #点球的决策树 目前有三种
flag_penalty = -1                 #如果为1 则换点球策略
defend_flag = 0           #0为正常防守  1为点球防守

goal_time = 0
time = 0                  #统计门球造犯规失败次数
goal = 1                  #门球的决策树 目前有两种
flag_goal = 0             #如果为1 则换门球策略
last_race_state = -1      #上一次比赛情况
last_race_state_trigger = -1  #上一次触发方
guard_goal = 1            #门球防守的决策树  防止被对方造犯规

last_futuerballx = 0
last_futuerbally = 0
tickBeginPenalty = 0
tickBeginGoalKick = 0
tickBeginPlaceKick = 0
lastBallx = -110 + 37.5
lastBally = 0
BallPos = [Vector2(0, 0)] * 100000
resetHistoryRecord = False
newMatch = False

# 打印比赛状态，详细请对比v5rpc.py
@unbox_event
def on_event(event_type: int, args: EventArguments):
    event = {
        0: lambda: print(args.judge_result.reason),
        1: lambda: print("Match Start"),
        2: lambda: print("Match Stop"),
        3: lambda: print("First Half Start"),
        4: lambda: print("Second Half Start"),
        5: lambda: print("Overtime Start"),
        6: lambda: print("Penalty Shootout Start"),
        7: lambda: print("MatchShootOutStart"),
        8: lambda: print("MatchBlockStart")
    }
    global race_state_trigger
    global race_state
    if event_type == 0:
        race_state = args.judge_result.type
        race_state_trigger = args.judge_result.offensive_team
        if race_state == JudgeResultEvent.ResultType.PlaceKick:
            print("Place Kick")
        elif race_state == JudgeResultEvent.ResultType.PenaltyKick:
            print("Penalty Kick")
        elif race_state == JudgeResultEvent.ResultType.GoalKick:
            print("Goal Kick")
        elif (race_state == JudgeResultEvent.ResultType.FreeKickLeftBot
              or race_state == JudgeResultEvent.ResultType.FreeKickRightBot
              or race_state == JudgeResultEvent.ResultType.FreeKickLeftTop
              or race_state == JudgeResultEvent.ResultType.FreeKickRightTop):
            print("Free Kick")

        actor = {
            Team.Self: lambda: print("By Self"),
            Team.Opponent: lambda: print("By Opp"),
            Team.Nobody: lambda: print("By Nobody"),
        }
        actor[race_state_trigger]()
    elif event_type == 6:
        race_state = 7 #点球大战

    event[event_type]()


@unbox_int
def get_team_info(server_version: int) -> str:
    version = {
        0: "V1.0",
        1: "V1.1"
    }
    print(f'server rpc version: {version.get(server_version, "V1.0")}')
    global newMatch
    newMatch = True
    return 'kimu队伍'# 在此行修改双引号中的字符串为自己的队伍名

'守门员函数'
def goalkeeper (field):
    global football_now_y,football_now_x
    global futureBally,futureBallx
    global defend_flag                   #点球防守flag

    football_now_x= field.ball.position.x
    football_now_y = field.ball.position.y 

    futureBallx = 8 * football_now_x - 7 * BallPos[GlobalVariable.tick - 1].x
    futureBally = 8 * football_now_y - 7 * BallPos[GlobalVariable.tick - 1].y


    if np.fabs(futureBally) <= 25: #球在球门y轴范围内

        if futureBallx < 0:  #球还在对方半场
            #  baseRobots[0].moveto_dis(GlobalVariable.goalkepper_X, 0)
            if football_now_y*0.8 + futureBally*0.2 >= 20:
                baseRobots[1].moveto_dis(GlobalVariable.goalkepper_X,  18)
            elif football_now_y*0.8 + futureBally*0.2 <= -20:
                baseRobots[1].moveto_dis(GlobalVariable.goalkepper_X,  -18)
            else:
                baseRobots[1].moveto(GlobalVariable.goalkepper_X, football_now_y*0.8 + futureBally*0.2)  

        else:   #球在我们半场 

                if  defend_flag == 1 :  #点球防守（不主动出去）
                    baseRobots[1].moveto (GlobalVariable.goalkepper_X,futureBally)
                # elif defend_flag == 2:  #点球防守 追求
                #     baseRobots[0].moveto (futureBallx,futureBally)                    
                else :                   #正常防守

                    if futureBallx <= 65:    #球还未到球门区

                        if futureBally >= 20:
                            baseRobots[1].moveto_dis(GlobalVariable.goalkepper_X,  18)
                        elif futureBally <= -20:
                            baseRobots[1].moveto_dis(GlobalVariable.goalkepper_X,  -18)
                        else:
                            baseRobots[1].moveto_dis(GlobalVariable.goalkepper_X, futureBally)
                    else:                    #x轴上球到球门区了           
                        num_in_goal = 0  
                        for i in range(0,5): #判断有没有对方球员进入我方守门区
                            if oppRobots[i].get_pos().x > 20 and  np.fabs(oppRobots[i].get_pos().y) < 25 and oppRobots[i].get_pos().x-football_now_x<5: #如果有  
                                num_in_goal = 1 +  num_in_goal 
                            
                        if num_in_goal != 0:    #对方有球员进入守门区
                            baseRobots[1].moveto (futureBallx,futureBally)
                        else:
                        #   if futureBallx >= 95 and np.fabs(futureBally)<25:  #球进球门区内部了就踢出去
                        #       baseRobots[0].moveto (futureBallx,futureBally)
                        #   else:
                            if futureBally >= 20:
                                baseRobots[1].moveto_dis(GlobalVariable.goalkepper_X,  18)
                            elif futureBally <= -20:
                                baseRobots[1].moveto_dis(GlobalVariable.goalkepper_X,  -18)
                            else:                        
                               baseRobots[1].moveto(GlobalVariable.goalkepper_X, futureBally*0.5+football_now_y*0.5) 

                            # if futureBallx >= baseRobots[0].get_pos ().x : #球x轴距离比守门员大则拦截
                            #     baseRobots[0].breakthrough (futureBallx,futureBally,futureBallx+2,futureBally)                        
                                        
    else:       #不在球门y轴范围
            if futureBallx <= 0:  #球在对方半场
                if futureBally > 20:
                    baseRobots[1].moveto_dis(GlobalVariable.goalkepper_X, 18)             
                elif futureBally < -20:
                    baseRobots[1].moveto_dis(GlobalVariable.goalkepper_X, -18) 
                else:
                    baseRobots[1].moveto_dis(GlobalVariable.goalkepper_X, futureBally)   
            elif futureBallx <= 60:
                    # if futureBally >= 30:
                    #     baseRobots[0].moveto_dis (GlobalVariable.goalkepper_X,15)    
                    # elif  futureBally <= -30:  
                    #     baseRobots[0].moveto_dis (GlobalVariable.goalkepper_X,-15)
                    # else:  #球门范围内
                        if futureBally >= 30:
                            baseRobots[1].moveto_dis (GlobalVariable.goalkepper_X,15)    
                        elif  futureBally <= -30:  
                            baseRobots[1].moveto_dis (GlobalVariable.goalkepper_X,-15)
                        else:  #球门范围内
                            baseRobots[1].moveto_dis(GlobalVariable.goalkepper_X, futureBally)  
            elif futureBallx <=95:          
                    if futureBally >= 20:
                        baseRobots[1].moveto_dis (GlobalVariable.goalkepper_X,18)    
                    elif  futureBally <= -20:  
                        baseRobots[1].moveto_dis (GlobalVariable.goalkepper_X,-18)
                    else:  #球门范围内
                        baseRobots[1].moveto(GlobalVariable.goalkepper_X, futureBally)                                                           
            else :  

                    baseRobots[1].moveto (futureBallx, futureBally)
                    # if futureBallx >= GlobalVariable.goalkepper_X : #球x轴距离比守门员大则拦截
                    #         baseRobots[0].breakthrough (futureBallx,futureBally,futureBallx+2,futureBally) 
                                                          

# 策略行为主函数，可将以下函数用策略模式封装
def strategy_common(field):      #正常的时候 
    # 最基本最常规情况下的执行策略
    global football_now_x,football_now_y

    football_now_x= field.ball.position.x
    football_now_y = field.ball.position.y    
    # 预测足球位置
    global futureBallx,futureBally

    futureBallx = 8 * football_now_x - 7 * BallPos[GlobalVariable.tick - 1].x
    futureBally = 8 * football_now_y - 7 * BallPos[GlobalVariable.tick - 1].y

    # print ('dis',baseRobots[1].PredictInformation[GlobalVariable.tick_delay].position.x,'tick',GlobalVariable.tick)
    # baseRobots[1].moveto_dis (110,20)
    #1号一直作为防守人员   2号控场  0号主要进攻
    '********************************防守大策略***********************************************************'                
    if football_now_x > 5:
        '0号主攻'
        # if baseRobots[1].PredictInformation[GlobalVariable.tick_delay].position.x > 85 or np.fabs(baseRobots[1].PredictInformation[GlobalVariable.tick_delay].position.y) < 25 + 3: #防止进入守门区

        #     # if np.fabs(baseRobots[1].get_pos().x - 85) > np.fabs(np.fabs(baseRobots[1].get_pos().y) - 25):  #离y轴近
        #     #     if baseRobots[1].get_pos().y > 0:                
        #     #         baseRobots[1].moveto (80,40)
        #     #     else :
        #             #  baseRobots[1].moveto (80,-40)                   
        #     # else :            #离x轴近
        #     baseRobots[1].moveto (80,baseRobots[1].get_pos().y)              
        # else :
        # if  baseRobots[1].get_pos ().x > football_now_x:
        if (futureBallx > 80 or np.fabs(futureBally) < 40) or (football_now_x > 80 or np.fabs(football_now_y) < 40): #防止进入守门区
            baseRobots[0].moveto_dis (65,futureBally*0.8)
        else :
            if futureBallx > baseRobots[1].get_pos ().x :           
                baseRobots[0].breakthrough(football_now_x,football_now_y,futureBallx+1,futureBally)
            else:
                baseRobots[0].shoot (futureBallx,futureBally,10)                            
        '0号主攻'

        '3号' 
        # print (baseRobots[3].PredictInformation[GlobalVariable.tick_delay].position.x)    
        if (futureBallx > 70.5 or np.fabs(futureBally) < 40) or ( football_now_x > 70 or np.fabs(football_now_y) < 40): #防止进入守门区
            baseRobots[3].moveto_dis (60,-70)
        else :
            if  baseRobots[0].get_pos().x > football_now_x and football_now_x > baseRobots[3].get_pos().x : #3号机器人挡住了0号主攻的路线就避开
                baseRobots[3].moveto( futureBallx, -futureBally)  
            else:
                if football_now_x > baseRobots[3].get_pos ().x :
                    baseRobots[3].breakthrough (football_now_x,football_now_y,futureBallx+1,futureBally) 
                else:
                    baseRobots[3].moveto (futureBallx,futureBally)                                                     
        '3号' 

        '4号'
        if (futureBallx > 70.5 or np.fabs(futureBally) < 40) or (football_now_x > 70 or np.fabs(football_now_y) < 40): #防止进入守门区
            baseRobots[2].moveto_dis (60,70)
        else :
            if football_now_x > baseRobots[2].get_pos ().x :
                baseRobots[2].breakthrough (football_now_x,football_now_y,futureBallx+1,futureBally) 
            else:
                baseRobots[2].moveto (futureBallx,futureBally)               

        '4号'

        '守门员专区' 
        goalkeeper (field)

        if futureBallx > baseRobots[4].get_pos ().x :
             baseRobots[4].move_in_still_x  (55,-futureBally) 
        else:      
            baseRobots[4].move_in_still_x  (55,football_now_y*0.8 + futureBally*0.2)  #2号后卫起一个拦截作用
            dx = football_now_x - baseRobots[4].get_pos().x
            dy = football_now_y - baseRobots[4].get_pos().y
            distance = math.sqrt(dx * dx + dy * dy)        
            if distance < 2 and dx > 0:
                if football_now_y < 0:
                    baseRobots[4].set_wheel_velocity(125, -125)
                else:
                    baseRobots[4].set_wheel_velocity(-125, 125)

    '********************************进攻大策略***********************************************************'
    if football_now_x <= 5:
        if  futureBallx+1 > -75 + 3 : #球还没到禁区          
            baseRobots[0].shoot (futureBallx+1,futureBally,10)
            baseRobots[3].move_with_angle(futureBallx+1, futureBally,180)  # 4号自由人追球
        else : #超过守门区了就踢球 防止冲撞守门员
            num_in_goal = 0
            for i in range(0,5): #判断有没有对方守门员
                if oppRobots[i].get_pos().x < -94 and  np.fabs(oppRobots[i].get_pos().y) < 30: #如果有  
                    num_in_goal += 1          

            if num_in_goal == 1:  #有守门员
                distance1_ball = math.sqrt((football_now_x - baseRobots[0].get_pos().x) ** 2 + (football_now_y - baseRobots[0].get_pos().y) ** 2)
                if futureBallx < -80 and distance1_ball < 10:  #在球门区而且球在跟前
                    if baseRobots[0].get_pos ().x <= football_now_x :  #0号在球的左侧                 
                         baseRobots[0].breakthrough (football_now_x,football_now_y,futureBallx+1,futureBally) 
                    else :
                        if football_now_y < 0:
                            baseRobots[0].set_wheel_velocity(125, -125)
                        else:
                            baseRobots[0].set_wheel_velocity(-125, 125) 
                else :
                    # baseRobots[0].shoot (futureBallx+1,futureBally,10)
                    baseRobots[0].move_with_angle(futureBallx+1, futureBally,180)
                    if (futureBallx<-90 and np.fabs(futureBally)<=60):
                        baseRobots[0].moveto_dis (-80,futureBally)

                if baseRobots[3].get_pos ().x <= football_now_x :  #0号在球的左侧 
                    baseRobots[3].breakthrough (football_now_x,football_now_y,futureBallx+1,futureBally)
                else:
                    baseRobots[3].move_with_angle(futureBallx+1, futureBally,180)
                distance3_ball = math.sqrt((futureBallx - baseRobots[3].get_pos().x) ** 2 + (futureBally - baseRobots[3].get_pos().y) ** 2)
                if (baseRobots[3].get_pos().x > futureBallx and distance3_ball<10):
                    if football_now_y < 0:
                        baseRobots[3].set_wheel_velocity(125, -125)
                    else:
                        baseRobots[3].set_wheel_velocity(-125, 125) 
                if (futureBallx<-90 and np.fabs(futureBally)<=50):
                    baseRobots[3].moveto_dis (-75,futureBally)             
            else :               #没有守门员
                baseRobots[0].shoot (futureBallx+1,futureBally,10)  
                # baseRobots[3].shoot(futureBallx, futureBally,10)

        '中场后卫'
        distance2_ball = math.sqrt((football_now_x - baseRobots[4].get_pos().x) ** 2 + (football_now_y - baseRobots[4].get_pos().y) ** 2)
        if distance2_ball < 5 and  (baseRobots[4].get_pos().x - football_now_x) > 0 : #求到眼前了就中场射门
            if football_now_y < 0:
                baseRobots[4].set_wheel_velocity(125, -125)
            else:
                baseRobots[4].set_wheel_velocity(-125, 125)            
        else :
                baseRobots[4].move_in_still_x( 45,  football_now_y*0.8 + futureBally*0.2)
                        
        distance4_ball = math.sqrt((football_now_x - baseRobots[2].get_pos().x) ** 2 + (football_now_y - baseRobots[2].get_pos().y) ** 2)
        if distance4_ball < 5 and  (baseRobots[2].get_pos().x - football_now_x) > 0 : #求到眼前了就中场射门
            if football_now_y < 0:
                baseRobots[2].set_wheel_velocity(125, -125)
            else:
                baseRobots[2].set_wheel_velocity(-125, 125)            
        else :
             baseRobots[2].move_in_still_x( 0,  futureBally)         
        '守门员专区' 
        goalkeeper (field) 


def strategy_penalty(field): #点球策略
    football_now_x = field.ball.position.x
    football_now_y = field.ball.position.y
    futureBallx = 8 * football_now_x - 7 * BallPos[GlobalVariable.tick - 1].x
    futureBally = 8 * football_now_y - 7 * BallPos[GlobalVariable.tick - 1].y
    global tickBeginPenalty
    global race_state_trigger
    global flag_penalty,penalty
    goalkeeper_num = 0
   
    if race_state_trigger == Team.Self:
        for i in range(0, 5):
            baseRobots[i].set_wheel_velocity(0, 0)
        # print(np.fabs(baseRobots[1].get_pos().x)) 
        if  penalty == 2 :   #点球策略1： 守门员定点射门#                   
            if GlobalVariable.tick - tickBeginPenalty <=30:        
              if   np.fabs(baseRobots[0].get_pos().x) < 64.5001:
                baseRobots[0].set_wheel_velocity(-125,-125)
              else:
               baseRobots[0].set_wheel_velocity(125,-125)
            else:
                flag_penalty = 1 #30拍没进球就是点球没进  换策略2
                strategy_common(field)
        elif penalty == 1:       #点球策略2： 守门员绕C字#
            if GlobalVariable.tick - tickBeginPenalty <= 8:
                baseRobots[0].set_wheel_velocity(-110, -77)
            elif GlobalVariable.tick - tickBeginPenalty <= 19:
                baseRobots[0].set_wheel_velocity(-90, -120)
            elif GlobalVariable.tick - tickBeginPenalty <= 28:
                baseRobots[0].set_wheel_velocity(-10, -125)
            elif GlobalVariable.tick - tickBeginPenalty <= 68:
                baseRobots[0].shoot(futureBallx, futureBally,10)
            else:
                flag_penalty = 1
                strategy_common(field)
        elif penalty == 4:    #3号战术，队友辅助射门
          if GlobalVariable.tick - tickBeginPenalty <= 65:
            baseRobots[3].set_wheel_velocity(80, 80)
            if  8 > np.fabs(baseRobots[4].get_pos().x) :
                baseRobots[0].set_wheel_velocity(80, 80)
                baseRobots[4].set_wheel_velocity(125,125)
            elif 8 < np.fabs(baseRobots[4].get_pos().x) < 31:
                baseRobots[4].set_wheel_velocity(105, 125)
                if 12 < np.fabs(baseRobots[4].get_pos().x) < 31:
                    baseRobots[0].set_wheel_velocity(40,20)  
            elif  50 > np.fabs(baseRobots[4].get_pos().x) > 31:
                baseRobots[0].set_wheel_velocity(65,45)
                baseRobots[4].set_wheel_velocity(125,125)
            elif  np.fabs(baseRobots[4].get_pos().x) > 50:
                baseRobots[4].set_wheel_velocity(125,-125)
          else:
                flag_penalty = 1 
                strategy_common(field)
        elif penalty == 3:    #4号战术，队友辅助射门
          if GlobalVariable.tick - tickBeginPenalty <= 55:
            if  10 > np.fabs(baseRobots[3].get_pos().x) :
                baseRobots[0].set_wheel_velocity(90, 90)
                baseRobots[3].set_wheel_velocity(125,125)
            elif 10 < np.fabs(baseRobots[3].get_pos().x) < 35:
                baseRobots[3].set_wheel_velocity(102.7, 125)
                if 12 < np.fabs(baseRobots[3].get_pos().x) < 35:
                    baseRobots[0].set_wheel_velocity(40,20)  
            elif 60 > np.fabs(baseRobots[3].get_pos().x) > 35:
                baseRobots[0].set_wheel_velocity(65,45)
                baseRobots[3].set_wheel_velocity(125,125)
          else:
                flag_penalty = 1 
                strategy_common(field)
        elif penalty == 5:    #点球策略5： 简易c字                                  
          if GlobalVariable.tick - tickBeginPenalty <=55:
             if  np.fabs(baseRobots[0].get_pos().x) < 69:
                baseRobots[0].set_wheel_velocity(91,95)
             elif  74 > np.fabs(baseRobots[0].get_pos().x) > 69:
                baseRobots[0].set_wheel_velocity(60,95)
             elif  np.fabs(baseRobots[0].get_pos().x) > 74:
                baseRobots[0].set_wheel_velocity(125,-125)
          else:
                flag_penalty = 1 
                strategy_common(field)
          
    if race_state_trigger == Team.Opponent:
        if GlobalVariable.tick - tickBeginPenalty <= 60 :
            baseRobots[0].moveto_dis(50, -15)
            baseRobots[2].moveto_dis(50, 15)
            baseRobots[3].moveto_dis(70, -65)
            baseRobots[4].moveto_dis(70, 65)
            goalkeeper (field)
        else:
            strategy_common(field) 

def strategy_goalkick(field):  #门球策略
    global tickBeginGoalKick
    global race_state_trigger
    global goal,guard_goal

    football_now_x = field.ball.position.x
    football_now_y = field.ball.position.y
    # 预测足球位置
    futureBallx = 8 * football_now_x - 7 * BallPos[GlobalVariable.tick - 1].x
    futureBally = 8 * football_now_y - 7 * BallPos[GlobalVariable.tick - 1].y       

    if race_state_trigger == Team.Self:
        if goal == 1:  #造犯规策略
            for i in range(2, 5):
                baseRobots[i].set_wheel_velocity(125, -125)
            baseRobots[0].moveto_dis(-50, 30)              
            baseRobots[1].set_wheel_velocity(0, 0)  #守门员 
        
        elif goal == 2:  #正常策略
            if GlobalVariable.tick - tickBeginGoalKick <= 75:
                for i in range(0, 5):
                    baseRobots[i].set_wheel_velocity(0, 0)
                baseRobots[1].set_wheel_velocity(125, 125)
            else:
                strategy_common(field)
        elif goal == 3: #开球直踢型策略
            for i in range(0, 5):
                baseRobots[i].set_wheel_velocity(0, 0)
            if GlobalVariable.tick - tickBeginGoalKick <= 30:
                 baseRobots[1].set_wheel_velocity(125, 125) 
            elif GlobalVariable.tick - tickBeginGoalKick <= 45:
                 baseRobots[1].set_wheel_velocity(125, -125)                  
            else:
                 strategy_common(field)            

    if race_state_trigger == Team.Opponent:
        if guard_goal == 1:
            if GlobalVariable.tick - tickBeginGoalKick <= 80:
                baseRobots[0].shoot (futureBallx,futureBally,10)               
                baseRobots[2].move_in_still_x  (65,futureBally)  #2号后卫起一个拦截作用
                baseRobots[3].moveto(25, futureBally) 
                baseRobots[4].moveto(50, futureBally)
                goalkeeper (field)               
            else:               
                strategy_common(field)
        elif guard_goal == 2:  #防止被对方造犯规
            goalkeeper (field)
            distance1_ball = math.sqrt((football_now_x - baseRobots[0].get_pos().x) ** 2 + (football_now_y - baseRobots[0].get_pos().y) ** 2)
            if futureBallx < -80 and distance1_ball < 10:  #在球门区而且球在跟前
                if baseRobots[0].get_pos ().x <= football_now_x and np.fabs(baseRobots[0].get_pos ().x-football_now_y)<1:  #0号在球的左侧                 
                    baseRobots[0].breakthrough (football_now_x,football_now_y,futureBallx+1,futureBally)  
                else :
                    if football_now_y < 0:
                        baseRobots[0].set_wheel_velocity(125, -125)
                    else:
                        baseRobots[0].set_wheel_velocity(-125, 125) 
            else :
                baseRobots[0].shoot (futureBallx,futureBally,10)
                if (futureBallx<-95 and np.fabs(futureBally)<25):
                    baseRobots[0].moveto_dis (-80,futureBally) 
            baseRobots[2].move_in_still_x  (65,futureBally*0.9)  #2号后卫起一个拦截作用
            baseRobots[3].moveto(25, football_now_y) 
            baseRobots[4].moveto(50, football_now_y)                                                  

def strategy_PlaceKick (field): #开球策略
    global tickBeginPlaceKick
    global race_state_trigger

    football_now_x = field.ball.position.x
    football_now_y = field.ball.position.y
    # 预测足球位置
    futureBallx = 8 * football_now_x - 7 * BallPos[GlobalVariable.tick - 1].x
    futureBally = 8 * football_now_y - 7 * BallPos[GlobalVariable.tick - 1].y 
    futureBally2 = 9 * football_now_y - 8 * BallPos[GlobalVariable.tick - 1].y


    if race_state_trigger == Team.Self:
        for i in range(2, 5):
            baseRobots[i].set_wheel_velocity(0, 0)
            if GlobalVariable.tick - tickBeginPlaceKick <= 100:
                if GlobalVariable.tick - tickBeginPlaceKick <= 27:
                    baseRobots[0].set_wheel_velocity (125,125)
                    baseRobots[3].set_wheel_velocity (125,125)
                    goalkeeper (field) 
                else:
                    strategy_common(field)                    
            else:
                    strategy_common(field)
    if race_state_trigger == Team.Opponent:
            if GlobalVariable.tick - tickBeginPlaceKick <= 135:
                strategy_common(field)      

                if futureBally > 35 :
                    baseRobots[4].moveto_dis(79,8)     
                elif futureBally < -35  :
                    baseRobots[4].moveto_dis(79,-8)  
                else:
                    baseRobots[4].moveto_dis(79,futureBally2)

                if  futureBallx > 55:
                    baseRobots[2].moveto (55,futureBally)                 
                    baseRobots[3].moveto (60,futureBally) 
                elif futureBallx < -65:
                    baseRobots[2].moveto (-60,futureBally)                 
                    baseRobots[3].moveto (-65,futureBally) 
                else:
                     if futureBallx < 0 :
                        baseRobots[2].moveto (futureBallx*1.8,30)                 
                        baseRobots[3].moveto (futureBallx*1.8,-30)  
                     else:
                        baseRobots[2].moveto (futureBallx,30)                 
                        baseRobots[3].moveto (futureBallx,-30)                         

            else:            
                strategy_common(field)


@unbox_field
def get_instruction(field: Field):  #策略接口
    # python start.py 20000    print(field.tick)  # tick从2起始
    GlobalVariable.tick = field.tick
    global resetHistoryRecord

    for i in range(0, 5): # 0 1 2 3 4
        baseRobots.append(BaseRobot()) 
        oppRobots.append(BaseRobot())
        baseRobots[i].update(field.self_robots[i], resetHistoryRecord)
        oppRobots[i].update(field.opponent_robots[i], resetHistoryRecord)
        global newMatch
        if field.tick == 2: #newMatch is True:
            for j in range(0, 8):
                baseRobots[i].HistoryInformation[j] = field.self_robots[i].copy()   # 第0拍主动维护历史数据
                baseRobots[i].PredictInformation[j] = field.self_robots[i].copy()	# 第0拍主动维护预测数据
            newMatch = False
        baseRobots[i].PredictRobotInformation(GlobalVariable.tick_delay)#(GlobalVariable.tick_delay)

    football_now_x = field.ball.position.x   # 蓝方假设
    football_now_y = field.ball.position.y

    field.ball.position.x = field.ball.position.x
    field.ball.position.y = field.ball.position.y

    global BallPos
    BallPos[GlobalVariable.tick] = Vector2(football_now_x, football_now_y)
    if resetHistoryRecord is True:
        for i in range(GlobalVariable.tick, GlobalVariable.tick - 11, -1):
            BallPos[i] = Vector2(football_now_x, football_now_y)
    #print (race_state)
    #根据情况执行什么样的策略
    if race_state == JudgeResultEvent.ResultType.PenaltyKick:
        strategy_penalty(field)  #点球策略 2
    elif race_state == JudgeResultEvent.ResultType.GoalKick:
        strategy_goalkick(field) #门球策略 1
    elif race_state == JudgeResultEvent.ResultType.PlaceKick:
        strategy_PlaceKick (field) #开球策略0
    else :
        strategy_common(field)


    for i in range(0, 5):
        baseRobots[i].save_last_information(football_now_x, football_now_y)
    data_loader.set_tick_state(GlobalVariable.tick, race_state)
    resetHistoryRecord = False

    velocity_to_set = []
    for i in range(0, 5):
        velocity_to_set.append((baseRobots[i].robot.wheel.left_speed, baseRobots[i].robot.wheel.right_speed))

    return velocity_to_set, 0    # 以第二元素的(0,1)表明重置开关,1表示重置


@unbox_field
def get_placement(field: Field) -> List[Tuple[float, float, float]]:  #返回摆球时的坐标 接口
    final_set_pos: List[Union[Tuple[int, int, int], Tuple[float, float, float]]]
    global resetHistoryRecord
    resetHistoryRecord = True
    global flag_penalty,penalty
    global flag_goal,goal
    global last_race_state,last_race_state_trigger
    global guard_goal
    global defend_flag,time

    defend_flag = 0
    global goal_time

    '点球决策树'
    if  flag_penalty == 1 :
        penalty = penalty + 1
        if penalty > 5:
            penalty = 1
    flag_penalty = -1
    '点球决策树'

    '门球决策树'
    if flag_goal == 1:  #进入门球摆位了置1
        #1.造犯规成功了继续造犯规策略  2.对方连续犯了5次
        if last_race_state == race_state or (race_state == JudgeResultEvent.ResultType.PlaceKick and race_state_trigger != Team.Self and goal_time>=3): 
            goal = 1
            goal_time = goal_time + 1
            if goal_time>=3:
                goal_time = 0
        #1.对方不犯规  2.其他没有造成功的情况
        else:    #两种点球策略谁进球了就继续
            if race_state == JudgeResultEvent.ResultType.PlaceKick and race_state_trigger != Team.Self: #上一种门球进球了
                # goal = goal‘
                flag_goal = 0
            else:
                goal = goal+1
                if goal >= 4:
                    if time >= 3:
                        goal = 2
                    else:
                        goal = 1
                time = time + 1

    flag_goal = 0

    #连续两次次触发对方门球 则说明对方在造犯规
    if (last_race_state == 1 and race_state == 1) and (last_race_state_trigger != Team.Self and race_state != Team.Self):
            guard_goal = 2
    else :
            guard_goal = 1

    '门球决策树'    

    if race_state == JudgeResultEvent.ResultType.PlaceKick:
        global tickBeginPlaceKick
        tickBeginPlaceKick = field.tick       
        if race_state_trigger == Team.Self:
            print("开球进攻摆位")
            set_pos = [
                [-7 ,-7.5, 55],   
                [GlobalVariable.goalkepper_X, 0, 90],#守门员             
                [5, 32, 180],
                [55.8, 61.5, -130],
                [50, 0, 180],       
                [0.0, 0.0, 0.0]]
            # set_pos = [(-103, 0, 90), (30, 0, 0), (-3, -10, 0), (-3, 10, 0), (-3, 0, 0), (0.0, 0.0, 0.0)]
        else:   # if race_state_trigger == Team.Opponent:
            print("开球防守摆位")
            set_pos = [            
                [30, 0, 180],
                [GlobalVariable.goalkepper_X, 0, 90],                           
                [7, 30, 180],
                [7, -30, 180],
                [79, 0, 180],
                [0.0, 0.0, 0.0]]
            # set_pos = [(-105, 0, 90), (10, 20, -90), (10, -20, -90), (10, 40, -90), (10, -40, -90), (0.0, 0.0, 0.0)]
    elif race_state == JudgeResultEvent.ResultType.PenaltyKick:
        global tickBeginPenalty
        tickBeginPenalty = field.tick
        if race_state_trigger == Team.Self:
            print("点球进攻摆位")
            if penalty == 2:#直接射门策略摆位
                set_pos = [
                       [-64.5,6,35],
                       [GlobalVariable.goalkepper_X, 0, 90],
                       [0, 50, 180],
                       [10, -50, 160],
                       [0, 0, -180],
                       [-5, -10, 0.0]]
            elif penalty == 1:   #复杂c字种策略摆位         
                set_pos = [
                         [-64, -1.2, -8.6],
                         [GlobalVariable.goalkepper_X, 0, 90],
                        [10, -70, 14],
                        [10, 60, -30],
                        [10, -60, 14],
                        [-5, 10, 0.0]]
            elif penalty == 4:#队友辅助射门策略摆位
                set_pos = [
                       [-76.5, -9, 70],
                       [GlobalVariable.goalkepper_X, 0, 90],
                       [5, 55, -180],
                       [5, 75, -165],
                       [5, 85,-165],
                       [-5, -10, 0.0]]     
            elif penalty == 3:#队友辅助射门策略摆位
                set_pos = [
                      [-76.5, -9, 70],
                      [GlobalVariable.goalkepper_X, 0, 90],
                       [0, 50, 180],
                       [5, 60,-170],
                       [0, 0, -180],
                       [-5, -10, 0.0]]     
            elif penalty == 5:#简易c字策略摆位
                set_pos = [
                       [-67.5, -8, 90],
                       [GlobalVariable.goalkepper_X, 0, 90],
                       [0, 50, 180],
                       [10, -50, 160],
                       [0, 0, -180],
                       [-5, -10, 0.0]]    
            
        else:   # if race_state_trigger == Team.Opponent:
            defend_flag = 1
            print("点球防守摆位")
            set_pos = [
                       [-10, 50,-40],
                       [GlobalVariable.goalkepper_X, 0, 180],
                       [-10, -40,40],
                       [-7, -65,-15],
                       [-7, 65, 15],
                       [0, 0.0, 0.0]]
    elif race_state == JudgeResultEvent.ResultType.GoalKick:
        global tickBeginGoalKick
        tickBeginGoalKick = field.tick
        if race_state_trigger == Team.Self:
            flag_goal = 1

            print("门球进攻摆位")
            if goal == 1:
                set_pos = [
                        [5, 85, -90],
                        [100, 9.05, 90],
                        [5, -85, 90],
                        [30, 85, -90],
                        [30, -85, 90],
                        [105, 9.05, 0.0]]
            elif goal == 2:
                set_pos = [
                        [50, -20, 90],
                        [102, 5.05, 90],
                        [50, -40, 90],
                        [30, 0, 90],
                        [50, 0, 90],
                        [104.5, 9.05, 0.0]] 
            elif goal == 3:
                 set_pos = [
                        [5, 85, 90],
                        [106, -4, 180],
                        [5, -80, 90],
                        [30, 80, 90],
                        [30, -80, 90],
                        [98, -4, 0.0]]                                
        else:   # if race_state_trigger == Team.Opponent:
            print("门球防守摆位")
            set_pos = [
                       [5, 0, 180],
                       [GlobalVariable.goalkepper_X, 0, 90],
                       [65, -40, 180],
                       [25, 40, 180],
                       [50, 0, 180],
                       [0.0, 0.0, 0.0]]
    elif (race_state == JudgeResultEvent.ResultType.FreeKickLeftTop
          or race_state == JudgeResultEvent.ResultType.FreeKickRightTop
          or race_state == JudgeResultEvent.ResultType.FreeKickRightBot
          or race_state == JudgeResultEvent.ResultType.FreeKickLeftBot):
        if race_state_trigger == Team.Self:
            print("争球进攻摆位")
            set_pos = [
                       [30, 0, 0],
                       [GlobalVariable.goalkepper_X, 0, 90],
                       [-3, -10, 0],
                       [-3, 10, 0],
                       [-3, 0, 0],
                       [0.0, 0.0, 0.0]]
        else:   # if race_state_trigger == Team.Opponent:
            print("争球防守摆位")
            set_pos = [
                       [30, 0, 0],
                       [GlobalVariable.goalkepper_X, 0, 90],
                       [10, -10, 0],
                       [10, 10, 0],
                       [10, 0, 0],
                       [0.0, 0.0, 0.0]]
    else:
        print("race_state = " + str(race_state))

    final_set_pos = [(set_pos[0][0], set_pos[0][1], set_pos[0][2]),
                     (set_pos[1][0], set_pos[1][1], set_pos[1][2]),
                     (set_pos[2][0], set_pos[2][1], set_pos[2][2]),
                     (set_pos[3][0], set_pos[3][1], set_pos[3][2]),
                     (set_pos[4][0], set_pos[4][1], set_pos[4][2]),
                     (set_pos[5][0], set_pos[5][1], set_pos[5][2])]

    last_race_state = race_state   #记录上一次比赛状态
    last_race_state_trigger  = race_state_trigger

    print(final_set_pos)
    return final_set_pos  # 最后一个是球位置（x,y,角）,角其实没用