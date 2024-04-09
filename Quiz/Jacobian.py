import numpy as np

def Jacobian(theta):
    th1 = theta[0] # 1つ目の関節の角度 [rad]
    th2 = theta[1] # 2つ目の関節の角度 [rad]
    th3 = theta[2] # 3つ目の関節の角度 [rad]
    L1 = 0.425
    L2 = 0.392
    L3 = 0.1
    
    J = np.zeros(shape=(3,3))
    # ここを考える #######################################
    J[0,0] = 0
    J[0,1] = 0
    J[0,2] = 0
    
    J[1,0] = 0
    J[1,1] = 0
    J[1,2] = 0
    
    J[2,0] = 0
    J[2,1] = 0
    J[2,2] = 0
    ########################################################
    return J









# 以下は描画用のコード

if __name__ == "__main__":
    # import matplotlib.pyplot as plt
    from ForwardKinematics import ForwardKinematics
    from hidden.plot import plot_to_rgba
    from PIL import Image
    import os
    
    
    # 初期角度
    theta = np.array([0,0,0])
    
    # 角速度
    d_theta = np.array([np.pi/20,np.pi/20,np.pi/20])
    
    # 初期位置・姿勢
    x = 0.917
    z = 0.163
    phi = 0
    
    Clear = "Clear"
    frames = []
    
    for i in range(100):
        t = i*0.05
        rgba, correct = plot_to_rgba(theta, x, z, phi, t)
        if correct == "Incorrect":
            Clear = "Failed"
        frames.append(rgba)
        pos, phi =ForwardKinematics(theta)
        
        J = Jacobian(theta)
        # J = np.zeros([3,3])
        # J = np.random.rand(3, 3)
        dx, dz, d_phi = J @ d_theta*0.05
        
        x = pos[0] + dx
        z = pos[2] + dz
        phi = phi + d_phi
        
        theta = theta + 0.05*d_theta

    images = [Image.fromarray(frame) for frame in frames]
    script_dir = os.path.dirname(os.path.abspath(__file__))
    directory = script_dir + "/gif"
    if not os.path.exists(directory):
        os.makedirs(directory)
    images[0].save(f"{directory}/Jacobian_check.gif", save_all=True, append_images=images[1:], optimize=False, duration=50, loop=0)

    print(Clear)


