import numpy as np

def ForwardKinematics(theta):
    th1 = theta[0] # 1つ目の関節の角度 [rad]
    th2 = theta[1] # 2つ目の関節の角度 [rad]
    th3 = theta[2] # 3つ目の関節の角度 [rad]
    L1 = 0.425
    L2 = 0.392
    L3 = 0.1
    h = 0.163
    
    # この部分を考える #######################
    x = 0   # 手先のx座標の値
    z = 0   # 手先のz座標の値
    phi = 0 # 手先の姿勢の値
    #########################################
    
    return np.array([x, 0.134, z]), phi






# 以下は描画用のコード

if __name__ == "__main__":
    import matplotlib.pyplot as plt
    from hidden.plot import sub_plot
    n_row = 3
    n_column = 3
    fig, axes = plt.subplots(n_row, n_column, tight_layout=True)
    fig.set_size_inches(15, 8)
    thetas = [np.array([0,0,0]),
              np.array([np.pi/3,np.pi/3,np.pi/3]),
              np.array([np.pi/3,0,np.pi/3]),
              np.array([np.pi/2,0,0]),
              np.array([np.pi/3,-np.pi/6,np.pi/3]),
              np.array([np.pi,0,0]),
              np.array([2*np.pi/3,-np.pi/2,0]),
              np.array([0,2*np.pi/3,-np.pi/2]),
              np.array([np.pi/4,-np.pi/3,-2*np.pi/3]),]
    
    for i in range(len(thetas)):
        theta = thetas[i]
        pos , phi = ForwardKinematics(theta)
        # pos = np.random.rand(3)
        # phi = np.random.rand(1)[0]
        correct, _x, _z, _phi = sub_plot(theta, pos[0], pos[2], phi, ax = axes[i//n_column,i%n_column])
        print("------------------------------------------------")
        print(f"Case{i+1}",f" theta {thetas[i]}")
        print(f"Your Answer    x:{pos[0]} z:{pos[2]} phi:{phi}")
        print(f"Correct Answer x:{_x} z:{_z} phi:{_phi}")
        print(correct)
        
    plt.show()
    
    
    
    
    
    