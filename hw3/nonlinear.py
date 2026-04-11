"""A file for EECE 5550 HW 3 Problem 3 - Nonlinear Control"""
import numpy as np
import matplotlib.pyplot as plt


l = 0.5
m = 0.75
mu = 0.25
g = 9.81

kp = 1
kd = -0.4

cond1 = (-mu+kd)**2/4/m/l**2 + g*m*l
cond2 = mu
print(f"Imag Cond: kp (={kp}) > {cond1:4.2f} and kd (={kd}) < {cond2:4.2f}")

cond3 = (-mu+kd)**2/4/m/l**2+g*m*l
cond4 = mu-2*m*l**2*np.sqrt(1/4*(-mu+kd)**2/m**2/l**4+(m*l-kp)/m/l**2)
print(f"Real Cond: kp (={kp}) > {cond3:4.2f} and kd (={kd}) < {cond4:4.2f}")



Ac = np.array([
    [0,1],
    [1/l - kp/m/l**2, (-mu+kd)/m/l**2]
])

print(f"Ac: {Ac}")
print(f"eig Ac: {np.linalg.eigvals(Ac)}")


k = 1000
minkp = 3
maxkp = 7
minkd = -2
maxkd = 2
kpvals = np.linspace(minkp,maxkp,k)
kdvals = np.linspace(minkd,maxkd,k)

kpmesh, kdmesh = np.meshgrid(kpvals,kdvals)


cond1 = (-mu+kdmesh)**2/4/m/l**2 + g*m*l
cond2 = mu
imagvalid = (kpmesh > cond1)

imagcond = np.full((k,k),fill_value=np.nan,dtype=float)
imagcond[imagvalid] = (kdmesh[imagvalid] < cond2)


realvalid = ~imagvalid
# cond3 = (mu+kdmesh[valid])**2/4/m/l**2+m*l
cond4 = mu-2*m*l**2*np.sqrt(1/4*(-mu+kdmesh[realvalid])**2/m**2/l**4+(g*m*l-kpmesh[realvalid])/m/l**2)

realcond = np.full_like(imagcond,fill_value=np.nan,dtype=float)
realcond[realvalid] = (kdmesh[realvalid] < cond4)

fullcond = (imagcond > 0) | (realcond > 0)




fig,axs = plt.subplots(1,3,figsize=(18,5))
fig.suptitle(f"Stability Regions for $l={l}$ and $m={m}$ and $\mu={mu}$")
extent = [minkp,maxkp,minkd,maxkd]
cmap = plt.cm.viridis.copy()
cmap.set_bad(color='gray')


im0 = axs[0].imshow(imagcond, extent=extent, origin='lower', cmap = cmap)
cbar0 = fig.colorbar(im0,ax=axs[0])
cbar0.set_ticks([0, 1])
cbar0.set_ticklabels(["Unstable", "Stable"])
axs[0].set_xlabel("$K_p$")
axs[0].set_ylabel("$K_d$")
axs[0].set_title("Stability when Underdamped")


im1 = axs[1].imshow(realcond, extent=extent, origin='lower', cmap = cmap)
cbar1 = fig.colorbar(im1,ax=axs[1])
cbar1.set_ticks([0, 1])
cbar1.set_ticklabels(["Unstable", "Stable"])
axs[1].set_xlabel("$K_p$")
axs[1].set_ylabel("$K_d$")
axs[1].set_title("Stability when Overdamped")

im2 = axs[2].imshow(fullcond, extent=extent, origin='lower', cmap = cmap)
cbar2 = fig.colorbar(im1,ax=axs[2])
cbar2.set_ticks([0, 1])
cbar2.set_ticklabels(["Unstable", "Stable"])
axs[2].set_xlabel("$K_p$")
axs[2].set_ylabel("$K_d$")
axs[2].set_title("Stability")

for ax in axs:
    mucolor = "lime"
    ax.plot([minkp,maxkp],[mu,mu],color=mucolor,linestyle='--')
    ax.text(minkp+0.1, mu+0.05,"$\mu$",color=mucolor)

    mlcolor = "orangered"
    ax.plot([g*m*l,g*m*l],[minkd,maxkd],color=mlcolor,linestyle='-.')
    ax.text(g*m*l+0.05,maxkd-0.2,"$gml$",color=mlcolor)

    rtcolor = "aqua"
    rtkpvals = (-mu+kdvals)**2/4/m/l**2 + g*m*l
    valid = rtkpvals < maxkp
    ax.plot(rtkpvals[valid],kdvals[valid],color=rtcolor,linestyle=':')
    ax.text(rtkpvals[round(2*k/3)]+0.1,kdvals[round(2*k/3)]-0.1,"Critically Damped",color=rtcolor)






k=1000
min1 = np.pi-1
max1 = np.pi+1
min2 = -1
max2 = 1
alpha = 1
x1vals = np.linspace(min1,max1,k)
x2vals = np.linspace(min2,max2,k)

x1mesh, x2mesh = np.meshgrid(x1vals,x2vals)

Vvals = -m*g*l*(1+np.cos(x1mesh)) + alpha*m*g*l*(1-np.cos(x1mesh)**2) + 1/2*m*l**2*x2mesh**2

dotVvals1 = 2*alpha*x2mesh*m*g*l*np.cos(x1mesh)*np.sin(x1mesh) - mu*x2mesh**2
dotVvals2 = -mu*x2mesh**2


fig,axs = plt.subplots(1,3,figsize=(18,5))
fig.suptitle(f"Lyapunov Function $V(x)$ (With $\\alpha={alpha}$)")
extent = [min1,max1,min2,max2]
cmap = plt.cm.inferno.copy()
cmap.set_bad(color='gray')

cmap2 = plt.cm.magma.copy()
cmap2.set_bad(color='gray')


im0 = axs[0].imshow(Vvals, extent=extent, origin='lower', cmap = cmap)
# im0 = axs[0].contour(Vvals, extent=extent, levels=10)
cbar0 = fig.colorbar(im0,ax=axs[0])
# cbar0.set_ticks([0, 1])
# cbar0.set_ticklabels(["Unstable", "Stable"])
axs[0].plot([np.pi],[0],marker="+",color="white")
axs[0].text(np.pi,0+0.05,"$x^*$",color="white")
axs[0].set_xlabel("$x_1$")
axs[0].set_ylabel("$x_2$")
axs[0].set_title("$V(x)$ Near $x^*$")


im1 = axs[1].imshow(dotVvals1, extent=extent, origin='lower', cmap = cmap2)
axs[1].contour(dotVvals1, extent=extent, levels=[0],colors="limegreen")
axs[1].text(min1+0.1,0+0.1,"$\\dot V(x) = 0$",color="limegreen")
cbar1 = fig.colorbar(im1,ax=axs[1])
# cbar0.set_ticks([0, 1])
# cbar0.set_ticklabels(["Unstable", "Stable"])
axs[1].plot([np.pi],[0],marker="+",color="black")
axs[1].text(np.pi+0.05,0+0.05,"$x^*$")
axs[1].set_xlabel("$x_1$")
axs[1].set_ylabel("$x_2$")
axs[1].set_title("$\\dot V(x)$ With $u=0$")

im2 = axs[2].imshow(dotVvals2, extent=extent, origin='lower', cmap = cmap2)
# axs[2].contour(dotVvals2, extent=extent, levels=[0],colors="limegreen")
cbar2 = fig.colorbar(im2,ax=axs[2])
# cbar0.set_ticks([0, 1])
# cbar0.set_ticklabels(["Unstable", "Stable"])
axs[2].plot([np.pi],[0],marker="+",color="black")
axs[2].text(np.pi,0+0.05,"$x^*$")
axs[2].set_xlabel("$x_1$")
axs[2].set_ylabel("$x_2$")
axs[2].set_title("$\\dot V(x)$ With $u(x)$")



x1 = 3.5
x2 = 0
print(-mu*x2)



plt.show()