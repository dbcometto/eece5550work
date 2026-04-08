"""A file for EECE 5550 HW 3 Problem 2 - PID Controller"""
import numpy as np
import matplotlib.pyplot as plt


MASS = 65e-3 # mass in kg
KT = 5.276e-4 # thrust coefficient (unitless)
G = 9.81 # gravity in m/s^2

def f(x: np.ndarray, u: float) -> np.ndarray:
    """Return dot x"""
    x2 = x[1]
    dotx = np.array([
        x2,
        4*KT*u/MASS - G
    ])

    return dotx

def u_p(x: np.ndarray, r: np.ndarray, kp: float) -> float:
    """Calculate the P controller value"""
    y = x[0]
    e = r-y

    return kp*e + MASS*G/(4*KT)

def u_pi(x: np.ndarray, r: np.ndarray, kp: float, i: float) -> float:
    """Calculate the P controller value"""
    y = x[0]
    e = r-y
    i+=e

    return kp*e + MASS*G/(4*KT), e



if __name__ == "__main__":
    t0 = 0
    tf = 20
    dt = 0.05
    r = 1
    kpvals = [5,15,50]


    fig, axs = plt.subplots(1,3,figsize=(18,6))
    fig.suptitle("P Controllers")

    for idx,kp in enumerate(kpvals):
        tvals = np.arange(t0,tf,dt)

        
        x = np.zeros((2,))
        xvals = []
        for t in tvals:
            xvals.append(x)
            u = u_p(x,r=r,kp=kp)
            xdot = f(x,u)
            x = x + dt*xdot
            

        yvals = [x[0] for x in xvals]
        axs[idx].plot(tvals,yvals)
        axs[idx].set_title(f"$K_p$: {kp}")
        axs[idx].set_xlabel("t")
        axs[idx].set_ylabel("y")
        axs[idx].grid()

    
    fig, axs = plt.subplots(1,3,figsize=(18,6))
    fig.suptitle("PI Controllers")

    for idx,kp in enumerate(kpvals):
        tvals = np.arange(t0,tf,dt)

        
        x = np.zeros((2,))
        i = 0
        xvals = []
        for t in tvals:
            xvals.append(x)
            u,i = u_pi(x,r=r,kp=kp,i=i)
            xdot = f(x,u)
            x = x + dt*xdot
            

        yvals = [x[0] for x in xvals]
        axs[idx].plot(tvals,yvals)
        axs[idx].set_title(f"$K_p$: {kp}")
        axs[idx].set_xlabel("t")
        axs[idx].set_ylabel("y")
        axs[idx].grid()

    plt.show()