# please do not change this file
import numpy as np
import matplotlib.pyplot as plt
from BuggySimulator import *


def wrap2pi(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


# initial your buggy
def initail(traj,n):
    v = vehicle(vehicle.state(X=traj[n, 0],
                              Y=traj[n, 1],
                              xd=0.1,
                              phi=np.arctan2(traj[n+1, 1] - traj[n, 1], traj[n+1, 0] - traj[n, 0])
                              ))
    return v


# vectorized find_nearest point
def closest_node(X, Y, traj):
    point = np.array([X, Y])
    traj = np.asarray(traj)
    dist = point - traj
    dist_2 = np.sum(dist ** 2, axis=1)
    minIndex = np.argmin(dist_2)
    return np.sqrt(dist_2[minIndex]), minIndex



# get the trajectory from .csv file
def get_trajectory(filename):
    with open(filename) as f:
        lines = f.readlines()
        traj = np.zeros((len(lines), 2))
        for idx, line in enumerate(lines):
            x = line.split(",")
            traj[idx, 0] = x[0]
            traj[idx, 1] = x[1]
    return traj


# save the states
def save_state(currentState):
    cur_state = [currentState.X,
                 currentState.Y,
                 currentState.delta,
                 currentState.xd,
                 currentState.yd,
                 currentState.phid,
                 currentState.phi]
    return cur_state


# show result
def showResult(traj,X,Y,delta,xd,yd,F,phi,phid,minDist):
    print('total steps: ', 0.05*len(X))

    fig, axes = plt.subplots(nrows=4, ncols=2,figsize=(15,10))

    plt.subplot(421)
    # plt.title('position')
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.plot(traj[:, 0], traj[:, 1], 'black',linewidth=6.0)
    plt.plot(X, Y, 'r')

    plt.subplot(422)
    # plt.title('delta')
    plt.xlabel('num steps')
    plt.ylabel('delta (rad)')
    plt.plot(delta, 'r')

    plt.subplot(423)
    # plt.title('xd')
    plt.ylabel('xd (m/s)')
    plt.xlabel('num steps')
    plt.plot(xd, 'r')

    plt.subplot(424)
    # plt.title('yd')
    plt.ylabel('yd (m/s)')
    plt.xlabel('num steps')
    plt.plot(yd, 'r')

    plt.subplot(425)
    # plt.title('phi')
    plt.xlabel('num steps')
    plt.ylabel('phi (rad)')
    plt.plot(phi, 'r')

    plt.subplot(426)
    # plt.title('phid')
    plt.ylabel('phid (rad/s)')
    plt.xlabel('num steps')
    plt.plot(phid, 'r')

    plt.subplot(427)
    plt.ylabel('minDist (m)')
    plt.xlabel('num steps')
    # plt.title('minDist')
    plt.plot(minDist, 'r')

    plt.subplot(428)
    # plt.title('F')
    plt.ylabel('F (N)')
    plt.xlabel('num steps')
    plt.plot(F, 'r')

    fig.tight_layout()

    avgDist = sum(minDist) / len(minDist)
    print('maxMinDist: ', max(minDist))
    print('avgMinDist: ', avgDist)
    plt.show()



if __name__ == "__main__":
    pass