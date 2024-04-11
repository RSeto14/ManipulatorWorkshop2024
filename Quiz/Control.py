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
        
        data.qpos[:] = np.array([np.pi/3, -np.pi/6, -np.pi/6])
        
        
        mujoco.mj_step(model, data)
        viewer.sync()

        time.sleep(2)
    
        start = time.time()
        
        while viewer.is_running():
            step_start = time.time()
            t = step_start - start
            
            
            
            theta = data.qpos.copy()
            d_theta = data.qvel.copy()
            
            
            torque, target_pos, target_phi = Control(t, theta, d_theta)
            
            model.body("target_body").pos[:] = target_pos
            model.body("target_body").quat[0] = np.cos(-target_phi/2)
            model.body("target_body").quat[2] = np.sin(-target_phi/2)
            
            data.ctrl[:] = torque
            
            mujoco.mj_step(model, data)
            viewer.sync()
            
            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)
            