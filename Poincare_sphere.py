import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.widgets import Button, Slider, RadioButtons
from functools import partial

# Define horizontally polarised light (theta = 0, phi = 0) and
# a HWP (eta = pi, varphi = 0) whose fast axes is oriented at
# (vartheta = ) 22.5° w.r.t. the optical table in the laboratory frame
q, p, e, vq, vp = 0, 0, np.pi, np.pi/8, 0

fig = plt.figure(constrained_layout = True)
ax = fig.add_subplot(111, projection='3d')

# Define sphere
u = np.linspace(0, 2 * np.pi, 100)
v = np.linspace(0, np.pi, 50)
x = np.outer(np.cos(u), np.sin(v))
y = np.outer(np.sin(u), np.sin(v))
z = np.outer(np.ones(np.size(u)), np.cos(v))

# Plot sphere
ax.plot_surface(x, y, z, color='purple', alpha=0.1, rstride=5, cstride=5)

# Remove axes
ax.set_axis_off()

# Define aspect ratio
ax.set_aspect('equal')

# Draw x, y, z axes from the origin using quiver
ax.quiver(-1, 0, 0, 2, 0, 0, color='k', arrow_length_ratio=0., lw = 0.5)  # x-axis
ax.quiver(0, -1, 0, 0, 2, 0, color='k', arrow_length_ratio=0, lw = 0.5)  # y-axis
ax.quiver(0, 0, -1, 0, 0, 2, color='k', arrow_length_ratio=0, lw = 0.5)  # z-axis

# Plot equator
equator_u = np.linspace(0, 2 * np.pi, 50)
equator_x = np.cos(equator_u)
equator_y = np.sin(equator_u)
equator_z = np.zeros_like(equator_u)
ax.plot(equator_x, equator_y, equator_z, color='k', ls = 'dotted', lw = 0.5)

# Plot prime meridian
meridian_v = np.linspace(0, 2 * np.pi, 50)
meridian_x = np.sin(meridian_v)
meridian_y = np.zeros_like(meridian_v)
meridian_z = np.cos(meridian_v)
ax.plot(meridian_x, meridian_y, meridian_z, color='k', ls = 'dotted', lw = 0.5)

# Plot other meridian at 90 degrees
meridian_x = np.zeros_like(meridian_v)
meridian_y = np.sin(meridian_v)
meridian_z = np.cos(meridian_v)
ax.plot(meridian_x, meridian_y, meridian_z, color='k', ls = 'dotted', lw = 0.5)

# Plot points (example points on the sphere)
points = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1], [-1, 0, 0], [0, -1, 0], [0, 0, -1]])
for point in points:
    ax.scatter(point[0], point[1], point[2], color='k', s=20)
    
# Point labels
ax.text(1.2, 0, 0, 'H', color='k')
ax.text(-1.1, 0, 0, 'V', color='k')
ax.text(0, 1.1, 0, 'D', color='k')
ax.text(0, -1.2, 0, 'A', color='k')
ax.text(0, 0, 1.1, 'R', color='k')
ax.text(0, 0, -1.2, 'L', color='k')

# Set plot label
ax.set_title('Wave plate in the Poincaré sphere') 

# Define colour notation (x, y, z)
fig.text(0.01, 0.9, r"Green: initial polarisation state defined by $\theta$ and $\phi$", color='g', fontsize=8)
fig.text(0.01, 0.85, r"Red: wave plate rotation axis defined by $\vartheta$", color='r', fontsize=8)
fig.text(0.01, 0.8, r"Blue: intermediate and final polarisation states", color='b', fontsize=8)

# Set viewing angle (elevation, azimuth)
ax.view_init(elev=30, azim=60)

# Define radial variable to plot vector lines
r = np.linspace(0,1,10)

# Define vector line coordinates
def vector(x, y, z):
    p = np.arctan2(y,x)
    q = np.arccos(z)
    xl = r*np.sin(q)*np.cos(p)
    yl = r*np.sin(q)*np.sin(p)
    zl = r*np.cos(q)
    return [xl, yl, zl]

# Define WP rotation axis
xwp = np.cos(2*vq)*np.cos(vp)
ywp = np.sin(2*vq)*np.cos(vp)
zwp = np.cos(np.pi/2-vp)
xwpl, ywpl, zwpl = vector(xwp, ywp, zwp)
point_wp, = ax.plot(xwp, ywp, zwp, ls = "", marker = "o", color='r', markersize = 6)
line_wp, = ax.plot(xwpl, ywpl, zwpl, ls = "-", color='r', lw = 2) 

# Get Stokes parameters S1, S2, and S3 from the Jones vector components alpha and beta after the action of a wave plate
def S(q, p, e, vq, vp):
    alpha = (np.cos(e/2)-1j*np.sin(e/2)*np.cos(2*vq))*np.cos(q/2)+(-np.sin(e/2)*np.sin(vp)*np.sin(2*vq)-1j*np.sin(e/2)*np.cos(vp)*np.sin(2*vq))*np.exp(1j*p)*np.sin(q/2)
    beta = (np.sin(e/2)*np.sin(vp)*np.sin(2*vq)-1j*np.sin(e/2)*np.cos(vp)*np.sin(2*vq))*np.cos(q/2)+(np.cos(e/2)+1j*np.sin(e/2)*np.cos(2*vq))*np.exp(1j*p)*np.sin(q/2)
    S1 = np.abs(alpha)**2-np.abs(beta)**2
    S2 = 2*np.real(alpha*np.conj(beta))
    S3 = 2*np.imag(alpha*np.conj(beta))
    return [S1, S2, S3]

# Define intermediate points
t = np.linspace(0,e,10)
points = []
for i in range(0,len(t)):
    x, y, z = S(q, p, t[i], vq, vp)   
    pts, = ax.plot(x, y, z, ls = "", marker = "o", color='b', markersize = 6)
    points.append(pts)

# Define initial Jones vector
xi, yi, zi = S(q, p, 0, 0, 0)
xil, yil, zil = vector(xi, yi, zi)
pointi, = ax.plot(xi, yi, zi, ls = "", marker = "o", color='g', markersize = 6)
linei, = ax.plot(xil, yil, zil, ls = "-", color='g', lw = 2)

# Define final Jones vector
xf, yf, zf = S(q, p, e, vq, vp)
xfl, yfl, zfl = vector(xf, yf, zf)
pointf, = ax.plot(xf, yf, zf, ls = "", marker = "o", color='b', markersize = 6)
linef, = ax.plot(xfl, yfl, zfl, ls = "-", color='b', lw = 2)

# Create sliders for q, p, and vq
ax_q = fig.add_axes([0.25, 0.15, 0.65, 0.05])
ax_p = fig.add_axes([0.25, 0.1, 0.65, 0.05])
ax_vq = fig.add_axes([0.25, 0.05, 0.65, 0.05])

qs = Slider(
    ax_q, r"$\theta$ (°)", 0., 180,
    valinit = 0, valstep = 1,
    color="green",
    initcolor='none',
    handle_style = {'facecolor': 'white', 'edgecolor': '0.5', 'size': 10}
)

ps = Slider(
    ax_p, r"$\phi$ (°)", -180, 180,
    valinit = 0, valstep = 2,
    color="green",
    initcolor='none',
    handle_style = {'facecolor': 'white', 'edgecolor': '0.5', 'size': 10}
)

vqs = Slider(
    ax_vq, r"$\vartheta$ (°)", -45, 45,
    valinit = 22.5, valstep = 0.5,
    color="red",
    initcolor='none',
    handle_style = {'facecolor': 'white', 'edgecolor': '0.5', 'size': 10}
)

# Define vectors and points update function
def update_intermediate_points(points, xf, yf, zf):
    points.set_data([xf], [yf])
    points.set_3d_properties(zf)

def update_point_and_line(point, line, xf, yf, zf, xfl, yfl, zfl):
    point.set_data([xf], [yf])
    point.set_3d_properties(zf)
    line.set_data(xfl, yfl)
    line.set_3d_properties(zfl)

def update(val, e):
    q = np.pi/180*qs.val
    p = np.pi/180*ps.val
    vq = np.pi/180*vqs.val

    # Define WP rotation axis
    xwp = np.cos(2*vq)*np.cos(vp)
    ywp = np.sin(2*vq)*np.cos(vp)
    zwp = np.cos(np.pi/2-vp)
    xwpl, ywpl, zwpl = vector(xwp, ywp, zwp)
    update_point_and_line(point_wp, line_wp, xwp, ywp, zwp, xwpl, ywpl, zwpl)

    # Define intermediate points
    t = np.linspace(0,e,10)
    for i in range(0,len(t)):
        x, y, z = S(q, p, t[i], vq, vp)
        update_intermediate_points(points[i], x, y, z)
    
    # Define initial Jones vector
    xi, yi, zi = S(q, p, 0, 0, 0)
    xil, yil, zil = vector(xi, yi, zi)
    update_point_and_line(pointi,linei,xi, yi, zi, xil, yil, zil)
    
    # Define final Jones vector
    xf, yf, zf = S(q, p, e, vq, vp)
    xfl, yfl, zfl = vector(xf, yf, zf)
    update_point_and_line(pointf,linef,xf, yf, zf, xfl, yfl, zfl)
    fig.canvas.draw_idle()         
        
# Set sliders action
qs.on_changed(partial(update, e = np.pi))
ps.on_changed(partial(update, e = np.pi))
vqs.on_changed(partial(update, e = np.pi))

# Create WP buttons
ax_WP = fig.add_axes([0.05, 0.07, 0.1, 0.1])
radio_WP = RadioButtons(ax_WP, labels = ['HWP', 'QWP'], active = 0, activecolor = 'r')

# Define WP update function
def updatePR(label):
    
    if label == 'HWP':
        qs.reset()
        ps.reset()
        vqs.reset()
        e = np.pi

        # Define intermediate points
        t = np.linspace(0,e,10)
        for i in range(0,len(t)):
            x, y, z = S(q, p, t[i], vq, vp)
            update_intermediate_points(points[i], x, y, z)

        # Define initial Jones vector
        xi, yi, zi = S(q, p, 0, 0, 0)
        xil, yil, zil = vector(xi, yi, zi)
        update_point_and_line(pointi,linei,xi, yi, zi, xil, yil, zil)
        
        # Define final Jones vector
        xf, yf, zf = S(q, p, e, vq, vp)
        xfl, yfl, zfl = vector(xf, yf, zf)
        update_point_and_line(pointf,linef,xf, yf, zf, xfl, yfl, zfl)
        fig.canvas.draw_idle()       

        # Set sliders action
        qs.on_changed(partial(update, e = np.pi))
        ps.on_changed(partial(update, e = np.pi))
        vqs.on_changed(partial(update, e = np.pi))
    
    elif label == 'QWP':
        qs.reset()
        ps.reset()
        vqs.reset()
        e = np.pi/2

        # Define intermediate points
        t = np.linspace(0,e,10)
        for i in range(0,len(t)):
            x, y, z = S(q, p, t[i], vq, vp)
            update_intermediate_points(points[i], x, y, z)
        
        # Define initial Jones vector
        xi, yi, zi = S(q, p, 0, 0, 0)
        xil, yil, zil = vector(xi, yi, zi)
        update_point_and_line(pointi,linei,xi, yi, zi, xil, yil, zil)
        
        # Define final Jones vector
        xf, yf, zf = S(q, p, e, vq, vp)
        xfl, yfl, zfl = vector(xf, yf, zf)
        update_point_and_line(pointf,linef,xf, yf, zf, xfl, yfl, zfl)
        fig.canvas.draw_idle()  

        # Set sliders action
        qs.on_changed(partial(update, e = np.pi/2))
        ps.on_changed(partial(update, e = np.pi/2))
        vqs.on_changed(partial(update, e = np.pi/2))

# Set WP buttons action
radio_WP.on_clicked(updatePR)

# Create reset button
ax_reset = fig.add_axes([0.8, 0.005, 0.1, 0.04])
button = Button(ax_reset, 'Reset', hovercolor='0.975')

# Define reset function
def reset(event):
    qs.reset()
    ps.reset()
    vqs.reset()

# Set reset button action
button.on_clicked(reset)

plt.show()
