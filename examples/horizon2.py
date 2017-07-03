import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Polygon
import numpy as np

import stader

d = stader.load_aircraft('b747_flight_condition2')
ac = stader.Aircraft(d)

msec = 15
dt = msec/1000

show_state = False
if show_state:
    fig = plt.figure(figsize=(16,8))
    ax = plt.subplot2grid((6,2), (0,0), rowspan=4)
    ax1 = plt.subplot2grid((6,2), (0,1))
    ax2 = plt.subplot2grid((6,2), (1,1))
    ax3 = plt.subplot2grid((6,2), (2,1))
    ax4 = plt.subplot2grid((6,2), (3,1))
    ax5 = plt.subplot2grid((6,2), (4,1))
    ax6 = plt.subplot2grid((6,2), (5,1))
    for ax_ in [ax1, ax2, ax3]:
        ax_.set_ylim([-180, 180])
    for ax_ in [ax1, ax2, ax3, ax4, ax5, ax6]:
        ax_.set_xlim([-60, 0])
else:
    fig, ax = plt.subplots()

ax.set_yticklabels([], visible=False)
ax.set_yticks([])
ax.set_xticklabels([], visible=False)
ax.set_xticks([])
ax.set_axis_bgcolor((30.0/255, 144.0/255, 1.0, 1))

ax.plot(0,0, marker='o', markersize=10, color='y')
ax.plot([-.5, -.25], [0, 0], marker=None, linestyle='-', linewidth=5, color='y')
ax.plot([.5, .25], [0, 0], marker=None, linestyle='-', linewidth=5, color='y')

ax.set_ylim(-1, 1)
ax.set_xlim(-1, 1)

gnd = Polygon(((-10,0), (10,0), (10,-10), (-10, -10)), closed=True,
              facecolor=(139.0/255, 69.0/255, 19.0/255),
              edgecolor='white')
gnd_xy = gnd.get_xy()
ax.add_artist(gnd)

markers = []
orig_xys = []
for tick in range(-50, 51, 5):
    x = 0.1 if tick % 10 else 0.25
    c = 'k' if tick > 0 else 'w'
    if tick == 0:
        continue
    markers.append(ax.plot([-x, x], [tick/35, tick/35], marker=None,
                           linestyle='-', linewidth=1, color=c)[0])
    orig_xys.append(markers[-1].get_xydata())

inputs = {'elevator':0, 'aileron':0}
tracking = False

def press(event):
    global inputs
    global tracking
    if event.key == 'up':
        inputs['elevator'] -= np.deg2rad(1)
    if event.key == 'down':
        inputs['elevator'] += np.deg2rad(1)
    if event.key == 'left':
        inputs['aileron'] -= np.deg2rad(1)
    if event.key == 'right':
        inputs['aileron'] += np.deg2rad(1)
    if event.key == 't':
        tracking = ~tracking
    print(inputs)

frame = 0

track_a = np.array([.5 if i < 6 else .05 for i in range(12)])
track_k = np.array([7, 11, 16, 25, 38, 61, 103, 131, 151, 181, 313, 523])
track_w = np.array([0.18, 0.28, 0.42, 0.65, 0.99, 1.60, 2.70, 3.43, 3.95, 4.74, 8.19, 13.69])
track_p = np.array([-0.29, -1.03, -3.13, 3.08, -0.84, 0.46, -2.74, -2.18, -1.78, -2.26, -1.82, 0.46])

def tracker(self):
    global frame
    frame += 1
    if tracking:
        t = frame*dt
        track = np.sum(track_a*2*np.pi*track_k/240.0*np.cos(2*np.pi*track_k*t/240.0 + track_p))
        track *= 0.025
        print(t, (track))
    else:
        track = 0

    ac.update(dt, inputs)

    gnd_center = (np.rad2deg(ac.pitch-track))/35

    R = np.array(((np.cos(ac.roll), np.sin(ac.roll), 0),
                  (-np.sin(ac.roll), np.cos(ac.roll), gnd_center),
                  (0,0,1)))

    xy = np.hstack((gnd_xy, np.ones((gnd_xy.shape[0], 1))))
    new_xy = R.dot(xy.T).T[:,:2]
    gnd.set_xy(new_xy)

    for orig_xy, marker in zip(orig_xys, markers):
        xy = np.hstack((orig_xy, np.ones((orig_xy.shape[0], 1))))
        new_xy = R.dot(xy.T)[:2,:]
        marker.set_data(new_xy)


anim = FuncAnimation(fig, tracker, interval=msec, blit=False, repeat=False)
fig.canvas.mpl_connect('key_press_event', press)
plt.show()
