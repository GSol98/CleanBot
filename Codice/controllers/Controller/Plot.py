from matplotlib.colors import ListedColormap
import matplotlib.pyplot as plt
import numpy as np


# ===============================================
#  plot delle posizioni del robot e dei landmark
# ===============================================

def plot_slam(mu, sigma):
    ax = plt.gca()
    theta = np.linspace(0, 2*np.pi, 100)

    ax.cla()

    a_bot = mu[1, 0] + sigma[1, 1] * 6 * np.cos(theta) * 30
    b_bot = -mu[0, 0] + sigma[0, 0] * 6 * np.sin(theta) * 30
    robot, = ax.plot(a_bot, b_bot, c='r', linewidth=1, label="robot")
    ax.plot(mu[1, 0], -mu[0, 0], c='r', marker='.', markersize=3)

    for j in range(3, len(mu)-1, 2):
        a = mu[j + 1, 0] + sigma[j + 1, j + 1] * 6 * np.cos(theta) * 30
        b = -mu[j, 0] + sigma[j, j] * 6 * np.sin(theta) * 30
        landmark, = ax.plot(a, b, c='k', linewidth=1, label="landmark")
        ax.plot(mu[j + 1, 0], -mu[j, 0], c='k', marker='.', markersize=3)
        ax.legend(handles=[robot, landmark], loc='best')

    plt.pause(0.02)


# ===============================================
#  plot degli ostacoli
# ===============================================

def plot_map(map):

    cmap = ListedColormap(['grey', 'w', 'k'])
    plt.matshow(map, fignum=0, cmap=cmap)
    plt.pause(0.02)
