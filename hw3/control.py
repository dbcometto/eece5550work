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

def u_pd(x: np.ndarray, r: np.ndarray, kp: float, kd: float) -> float:
    """Calculate the P controller value"""
    y = x[0]
    e = r-y
    dot_e = -x[1]

    return kp*e + kd*dot_e + MASS*G/(4*KT)

def u_pid(x: np.ndarray, r: np.ndarray, kp: float, kd: float, ki: float, accum_e: float) -> tuple[float]:
    """Calculate the P controller value"""
    y = x[0]
    e = r-y
    dot_e = -x[1]

    return kp*e + kd*dot_e + ki*accum_e + MASS*G/(4*KT), accum_e + e



if __name__ == "__main__":

    figw = 6
    figh = 4

    # P Plotting

    t0 = 0
    tf = 20
    dt = 0.05
    r = 1
    kpvals = [5,15,50]

    fig, axs = plt.subplots(1,3,figsize=(3*figw,figh))
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
        dothvals =  [x[1] for x in xvals]
        axs[idx].plot(tvals,yvals,label="$h$")
        axs[idx].plot(tvals,dothvals,label="$\\dot{h}$")
        axs[idx].plot([t0, tf], [r,r],linestyle="--",label="r")
        axs[idx].set_title(f"$K_p$: {kp}")
        axs[idx].set_xlabel("t")
        axs[idx].set_ylabel("y")
        axs[idx].grid()
        axs[idx].legend()
        axs[idx].set_xlim([t0,tf])

    



    # PD plotting

    fig, axs = plt.subplots(1,2,figsize=(2*figw,figh))
    fig.suptitle("PD Controllers")

    t0 = 0
    tf = 8
    dt = 0.05
    r = 1
    zetavals = [0.5, 1.1]

    for idx,zeta in enumerate(zetavals):
        tvals = np.arange(t0,tf,dt)

        wn = 4/zeta/3
        kp = wn**2*MASS/4/KT
        kd = zeta*wn*MASS/2/KT
  
        x = np.zeros((2,))
        xvals = []
        for t in tvals:
            xvals.append(x)
            u = u_pd(x,r=r,kp=kp,kd=kd)
            xdot = f(x,u)
            x = x + dt*xdot
        
        yvals = [x[0] for x in xvals]
        dothvals =  [x[1] for x in xvals]
        axs[idx].plot(tvals,yvals,label="$h$")
        axs[idx].plot(tvals,dothvals,label="$\\dot{h}$")
        axs[idx].plot([t0, tf], [r,r],linestyle="--",label="r")
        axs[idx].set_title(f"$K_p$: {kp:4.2f} and $K_d$: {kd:4.2f}")
        axs[idx].set_xlabel("t")
        axs[idx].set_ylabel("y")
        axs[idx].grid()
        axs[idx].legend()
        axs[idx].set_xlim([t0,tf])




    # PID

    t0 = 0
    tf = 20
    dt = 0.05
    r = 1
    zetavals = [0.5, 1.1]
    ki = 1

    fig, axs = plt.subplots(1,2,figsize=(2*figw,figh))
    fig.suptitle("Disturbed PD Controllers")

    for idx,zeta in enumerate(zetavals):
        tvals = np.arange(t0,tf,dt)

        wn = 4/zeta/3
        kp = wn**2*MASS/4/KT
        kd = zeta*wn*MASS/2/KT
  
        x = np.zeros((2,))
        xvals = []
        accum_e = 0
        for t in tvals:
            xvals.append(x)
            u = u_pd(x,r=r,kp=kp,kd=kd)

            u_true = 0.95*u
            xdot = f(x,u_true)
            x = x + dt*xdot
        
        yvals = [x[0] for x in xvals]
        dothvals =  [x[1] for x in xvals]
        axs[idx].plot(tvals,yvals,label="$h$")
        axs[idx].plot(tvals,dothvals,label="$\\dot{h}$")
        axs[idx].plot([t0, tf], [r,r],linestyle="--",label="r")
        axs[idx].set_title(f"Disturbed, $K_p$: {kp:4.2f} and $K_d$: {kd:4.2f}")
        axs[idx].set_xlabel("t")
        axs[idx].set_ylabel("y")
        axs[idx].grid()
        axs[idx].legend()
        axs[idx].set_xlim([t0,tf])


    fig, axs = plt.subplots(1,2,figsize=(2*figw,figh))
    fig.suptitle(f"PID Controllers: $K_i$: {ki}")

    for idx,zeta in enumerate(zetavals):
        tvals = np.arange(t0,tf,dt)

        wn = 4/zeta/3
        kp = wn**2*MASS/4/KT
        kd = zeta*wn*MASS/2/KT
  
        x = np.zeros((2,))
        xvals = []
        accum_e = 0
        for t in tvals:
            xvals.append(x)
            u,accum_e = u_pid(x,r=r,kp=kp,kd=kd,ki=ki,accum_e=accum_e)

            u_true = 0.95*u
            xdot = f(x,u_true)
            x = x + dt*xdot
        
        yvals = [x[0] for x in xvals]
        dothvals =  [x[1] for x in xvals]
        axs[idx].plot(tvals,yvals,label="$h$")
        axs[idx].plot(tvals,dothvals,label="$\\dot{h}$")
        axs[idx].plot([t0, tf], [r,r],linestyle="--",label="r")
        axs[idx].set_title(f"$K_p$: {kp:4.2f} and $K_d$: {kd:4.2f}")
        axs[idx].set_xlabel("t")
        axs[idx].set_ylabel("y")
        axs[idx].grid()
        axs[idx].legend()
        axs[idx].set_xlim([t0,tf])

    plt.show()