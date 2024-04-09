import numpy as np

def Target(t):
    
    # 初期位置・姿勢
    init_x = 0.65198196      
    init_z = 0.7270608
    init_phi = 0
    
    # ここを考える #################
    # x,z,phi(任意)を時刻 t[s] の関数で表す。
    x = init_x
    z = init_z
    phi = init_phi
    ###############################
    
    return np.array([x, 0.134, z]), phi








if __name__ == "__main__":
    from hidden.plot import plot_arrow_to_rgba
    from PIL import Image
    import os
    
    frames = []
    
    for i in range(100):
        t = 0.05*i
        pos, phi = Target(t)
        rgba = plot_arrow_to_rgba(pos[0],pos[2], phi, t)
        frames.append(rgba)

    images = [Image.fromarray(frame) for frame in frames]
    script_dir = os.path.dirname(os.path.abspath(__file__))
    directory = script_dir + "/gif"
    if not os.path.exists(directory):
        os.makedirs(directory)
    images[0].save(f"{directory}/Trajectory_check.gif", save_all=True, append_images=images[1:], optimize=False, duration=50, loop=0)