import numpy as np

from ForwardKinematics import ForwardKinematics #Quiz1 で作成した関数
from Jacobian import Jacobian #Quiz2 で作成した関数
from Trajectory import Target #Quiz3 で作成した関数

def Control(t, theta, d_theta):
    # ゲインを調整#########
    Kp = 0
    Kd = 0
    ######################
    
    
    pos,phi = ForwardKinematics(theta)
    J = Jacobian(theta)
    target_pos, target_phi = Target(t)
    
    diff_pos = target_pos - pos
    diff_phi = target_phi - phi
    
    
    torque = Kp*(J.T @ np.array([diff_pos[0],diff_pos[2], diff_phi])) - Kd* d_theta
    
    return torque, target_pos, target_phi








# 以下は描画用のコード

if __name__ == "__main__":
    import mujoco
    import mujoco.viewer
    import os
    import time
    
    script_dir = os.path.dirname(os.path.abspath(__file__))
    model = mujoco.MjModel.from_xml_path(f"{script_dir}/universal_robots_ur5e/scene.xml")
    data = mujoco.MjData(model)
    
    with mujoco.viewer.launch_passive(model, data) as viewer:
    # Close the viewer automatically after 30 wall-seconds.
        
        data.joint("shoulder_lift_joint").qpos = np.pi/3
        data.joint("elbow_joint").qpos = -np.pi/6
        data.joint("wrist_1_joint").qpos = -np.pi/6
        
        
        mujoco.mj_step(model, data)
        viewer.sync()

        time.sleep(2)
    
        start = time.time()
        
        while viewer.is_running():
            step_start = time.time()
            t = step_start - start
            
            
            
            theta = np.array([data.joint("shoulder_lift_joint").qpos[0], data.joint("elbow_joint").qpos[0], data.joint("wrist_1_joint").qpos[0]])
            d_theta = np.array([data.joint("shoulder_lift_joint").qvel[0], data.joint("elbow_joint").qvel[0], data.joint("wrist_1_joint").qvel[0]])
            
            
            
            torque, target_pos, target_phi = Control(t, theta, d_theta)
            
            data.joint("target_joint").qpos[0:3] = target_pos
            data.joint("target_joint").qpos[3:7] = np.array([np.cos(-target_phi/2),0,np.sin(-target_phi/2),0])
            
            data.ctrl[:] = torque
            
            mujoco.mj_step(model, data)
            viewer.sync()
            
            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)
            