# liuyiyuan

diaasiaiiiodsjio

faojoafon
<video src="./8.mp4" controls width="100%"></video>
```import pybullet as p
import pybullet_data
import time
import math

# --- 环境初始化 ---
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)
p.resetDebugVisualizerCamera(1.5, 45, -30, [0.5, 0, 0.65])

p.loadURDF("plane.urdf")
p.loadURDF("table/table.urdf", [0.5, 0, 0], useFixedBase=True)

# 1. 加载机器人
pandaId = p.loadURDF("franka_panda/panda.urdf", [0.5, 0, 0.625], useFixedBase=True)

# 2. 修改方块位置 (这里改成了 x=0.6, y=-0.2)
target_pos = [0.6, -0.2, 0.66]
cubeId = p.loadURDF("cube_small.urdf", target_pos)

# 增加摩擦力，否则容易滑掉
p.changeDynamics(cubeId, -1, lateralFriction=5.0)
p.changeDynamics(pandaId, 9, lateralFriction=5.0)
p.changeDynamics(pandaId, 10, lateralFriction=5.0)

def move_ee(pos):
    # 计算逆向运动学，保持夹爪垂直向下 (math.pi, 0, 0)
    jointPoses = p.calculateInverseKinematics(
        pandaId, 11, pos, 
        p.getQuaternionFromEuler([math.pi, 0, 0])
    )
    for i in range(7):
        p.setJointMotorControl2(pandaId, i, p.POSITION_CONTROL, jointPoses[i], force=500)

def gripper(opening):
    p.setJointMotorControl2(pandaId, 9, p.POSITION_CONTROL, opening, force=200)
    p.setJointMotorControl2(pandaId, 10, p.POSITION_CONTROL, opening, force=200)

state = 0
state_t = time.time()

while True:
    now = time.time()
    dt = now - state_t
    cube_pos, _ = p.getBasePositionAndOrientation(cubeId)

    if state == 0: # 移动到方块上方 15cm
        move_ee([cube_pos[0], cube_pos[1], cube_pos[2] + 0.15])
        gripper(0.04) 
        if dt > 1.5: state = 1; state_t = now

    elif state == 1: # 下降：注意 z 偏移量设为 +0.01 确保夹爪深度足够
        move_ee([cube_pos[0], cube_pos[1], cube_pos[2] + 0.01])
        if dt > 1.0: state = 2; state_t = now

    elif state == 2: # 闭合夹爪
        gripper(0.0) 
        if dt > 1.0: state = 3; state_t = now

    elif state == 3: # 提起来
        # 此时即便方块坐标在变，我们依然保持在它原本位置的上方
        move_ee([cube_pos[0], cube_pos[1], 0.95])
        gripper(0.0) # 提起来的过程中必须持续给闭合力

    p.stepSimulation()
    time.sleep(1./240.)