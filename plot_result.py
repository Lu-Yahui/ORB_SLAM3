import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def load_keyframe_traj(filename):
    xs, ys, zs = [], [], []
    with open(filename, "r") as f:
        for line in f:
            values = line.strip().split(" ")
            x = float(values[1])
            y = float(values[2])
            z = float(values[3])
            xs.append(x)
            ys.append(y)
            zs.append(z)
    return xs, ys, zs

def load_map_points(filename):
    xs, ys, zs = [], [], []
    with open(filename, "r") as f:
        for line in f:
            values = line.strip().split(" ")
            x = float(values[0])
            y = float(values[1])
            z = float(values[2])
            xs.append(x)
            ys.append(y)
            zs.append(z)
    return xs, ys, zs

if __name__ == "__main__":
    traj_xs, traj_ys, traj_zs = load_keyframe_traj("KeyFrameTrajectory.txt")
    xs, ys, zs = load_map_points("map.txt")

    fig = plt.figure(figsize=(4,4))
    ax = fig.add_subplot(111, projection='3d')
    # ax.scatter(xs, ys, zs)
    ax.plot(xs, ys, zs, ".")
    ax.plot(traj_xs, traj_ys, traj_zs)
    plt.show()