import numpy as np
import matplotlib.pyplot as plt

def joint_x_z_pos(theta):
    th1 = theta[0]
    th2 = theta[1]
    th3 = theta[2]
    L1 = 0.425
    L2 = 0.392
    L3 = 0.1
    h  = 0.163
    x = np.array([0,
                  0,
                  L1*np.cos(th1),
                  L1*np.cos(th1) + L2*np.cos(th1+th2),
                  L1*np.cos(th1) + L2*np.cos(th1+th2) + L3*np.cos(th1+th2+th3)])
    z = np.array([0,
                  h,
                  L1*np.sin(th1) + h,
                  L1*np.sin(th1) + L2*np.sin(th1+th2) + h,
                  L1*np.sin(th1) + L2*np.sin(th1+th2) + L3*np.sin(th1+th2+th3) + h])
    
    phi = th1 + th2 + th3
    return x, z, phi

def sub_plot(theta, _x, _z, _phi, ax):
    x, z, phi =joint_x_z_pos(theta)
    correct = "Correct" if np.all(np.array([x[4],z[4],phi]) - np.array([_x,_z,_phi]) < 0.01 )else "Incorrect"
    
    if correct == "Correct":
        color = "green"
    else:
        color = "red"
    
    
    ax.set_xlim([-1.1, 1.1])
    ax.set_ylim([-0.0, 1.2])
    ax.set_aspect('equal')
    ax.plot(x, z, marker='o', color="lightblue", linewidth=3, markersize=10,  zorder=0)
    ax.plot(_x, _z, marker='o', markersize=5, color=color, zorder=1)
    ax.arrow(_x, _z, 0.07*np.cos(_phi), 0.07*np.sin(_phi), color=color, head_width=0.02, head_length=0.02,  zorder=1)
    
    
    return correct, x[4], z[4], phi
    
def plot_to_rgba(theta, _x, _z, _phi, t):
    fig, ax = plt.subplots()
    ax.set_title(f"{round(t,1)}[s]",loc="left",fontsize=10)
    correct, _, _, _ = sub_plot(theta, _x, _z, _phi, ax)
    
    fig.canvas.draw()
    rgba = np.array(fig.canvas.renderer.buffer_rgba())
    plt.close(fig)
    
    return rgba, correct

def plot_arrow_to_rgba(x, z, phi, t):
    fig, ax = plt.subplots()
    ax.set_xlim([-1.1, 1.1])
    ax.set_ylim([-0.0, 1.2])
    ax.set_aspect('equal')
    ax.plot(x, z, marker='o', markersize=5, color="green", zorder=1)
    ax.arrow(x, z, 0.07*np.cos(phi), 0.07*np.sin(phi), color="green", head_width=0.02, head_length=0.02,  zorder=1)
    ax.set_title(f"{round(t,1)}[s]",loc="left",fontsize=10)
    fig.canvas.draw()
    
    rgba = np.array(fig.canvas.renderer.buffer_rgba())
    plt.close(fig)
    
    return rgba


    
    
    
if __name__ == "__main__":
    
    fig, axes = plt.subplots(2, 8, tight_layout=True)
    fig.set_size_inches(15, 8)
    sub_plot(np.array([0,0,0]), 0.5, 0.3, np.pi/3,ax = axes[0,0])
    plt.show()