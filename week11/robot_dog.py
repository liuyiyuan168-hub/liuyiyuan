import pybullet as p
import pybullet_data
import time
import numpy as np
import math


class QuadrupedController:
    """四足机器人控制器"""

    def __init__(self, robot_id):

        self.robot_id = robot_id

        # ==========================================
        # 正确的 Laikago 关节
        # ==========================================
        self.leg_joints = {

            # Front Left
            'LF': [0, 1, 2],

            # Front Right
            'RF': [4, 5, 6],

            # Rear Left
            'LH': [8, 9, 10],

            # Rear Right
            'RH': [12, 13, 14]
        }

        # ==========================================
        # 基础站立姿态
        # ==========================================
        self.base_pose = [0.0, 0.67, -1.3]

    # ==========================================
    # 稳定站立
    # ==========================================
    def stand(self):

        for leg_name, joint_ids in self.leg_joints.items():

            hip = self.base_pose[0]
            thigh = self.base_pose[1]
            calf = self.base_pose[2]

            target_angles = [hip, thigh, calf]

            for joint_id, angle in zip(joint_ids, target_angles):

                p.setJointMotorControl2(

                    bodyUniqueId=self.robot_id,

                    jointIndex=joint_id,

                    controlMode=p.POSITION_CONTROL,

                    targetPosition=angle,

                    force=220,

                    positionGain=0.8,

                    velocityGain=0.3
                )

    # ==========================================
    # 慢速前进步态
    # ==========================================
    def walk_gait(self, t, leg_name):

        # 对角腿同步
        if leg_name in ['LF', 'RH']:
            phase = 0
        else:
            phase = np.pi

        # 慢速步态
        cycle = (2 * np.pi * 0.8 * t + phase) % (2 * np.pi)

        # 默认站立
        hip = 0.0
        thigh = 0.67
        calf = -1.3

        # ==========================================
        # 抬腿阶段
        # ==========================================
        if cycle < np.pi:

            progress = cycle / np.pi

            # 抬腿
            thigh += 0.15 * np.sin(np.pi * progress)

            # 小腿摆动
            calf += 0.25 * np.sin(np.pi * progress)

            # 身体前摆
            hip += 0.05

        # ==========================================
        # 支撑阶段
        # ==========================================
        else:

            hip -= 0.05

        return [hip, thigh, calf]

    # ==========================================
    # 行走控制
    # ==========================================
    def walk(self, t):

        for leg_name, joint_ids in self.leg_joints.items():

            target_angles = self.walk_gait(
                t,
                leg_name
            )

            for joint_id, angle in zip(joint_ids, target_angles):

                p.setJointMotorControl2(

                    bodyUniqueId=self.robot_id,

                    jointIndex=joint_id,

                    controlMode=p.POSITION_CONTROL,

                    targetPosition=angle,

                    force=220,

                    positionGain=0.8,

                    velocityGain=0.3
                )


# ==========================================
# 主程序
# ==========================================
def main():

    # ==========================================
    # 初始化
    # ==========================================
    p.connect(p.GUI)

    p.setAdditionalSearchPath(
        pybullet_data.getDataPath()
    )

    p.setGravity(0, 0, -9.8)

    p.setPhysicsEngineParameter(
        numSolverIterations=200
    )

    # ==========================================
    # 地面
    # ==========================================
    p.loadURDF("plane.urdf")

    # ==========================================
    # 保持你的 orientation
    # 不修改
    # ==========================================
    start_orientation = p.getQuaternionFromEuler(
        [math.pi / 2, 0, math.pi / 2]
    )

    robotId = p.loadURDF(

        "laikago/laikago_toes.urdf",

        [0, 0, 0.5],

        start_orientation
    )

    # ==========================================
    # 增加摩擦
    # ==========================================
    p.changeDynamics(

        robotId,
        -1,

        lateralFriction=1.5,
        spinningFriction=0.1,
        rollingFriction=0.1
    )

    # ==========================================
    # 关闭默认电机
    # ==========================================
    for j in range(p.getNumJoints(robotId)):

        p.setJointMotorControl2(

            robotId,
            j,

            controlMode=p.VELOCITY_CONTROL,

            force=0
        )

    # ==========================================
    # 初始化站立姿态
    # ==========================================
    init_joint_ids = [

        0, 1, 2,
        4, 5, 6,
        8, 9, 10,
        12, 13, 14
    ]

    init_angles = [

        0.0, 0.67, -1.3,
        0.0, 0.67, -1.3,
        0.0, 0.67, -1.3,
        0.0, 0.67, -1.3
    ]

    for i in range(12):

        p.resetJointState(

            robotId,

            init_joint_ids[i],

            init_angles[i]
        )

    # ==========================================
    # 相机
    # ==========================================
    p.resetDebugVisualizerCamera(

        cameraDistance=2.0,

        cameraYaw=45,

        cameraPitch=-20,

        cameraTargetPosition=[0, 0, 0.3]
    )

    # ==========================================
    # 创建控制器
    # ==========================================
    controller = QuadrupedController(robotId)

    dt = 1. / 240.

    print("机器人站立中...")

    # ==========================================
    # 先站立 3 秒
    # ==========================================
    for _ in range(3 * 240):

        controller.stand()

        p.stepSimulation()

        time.sleep(dt)

    print("开始慢慢向前走...")

    # ==========================================
    # 开始行走
    # ==========================================
    t = 0

    while True:

        controller.walk(t)

        p.stepSimulation()

        time.sleep(dt)

        t += dt


# ==========================================
# 程序入口
# ==========================================
if __name__ == '__main__':

    main()