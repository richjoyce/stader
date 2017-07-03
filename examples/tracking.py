import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import stader
import numpy as np

d = stader.read_json('b747_flight_condition2.json')
d = stader.calculate_stability(d)

ac = stader.AircraftLongitudinal(d)
msec = 15
dt = msec/1000

fig, ax = plt.subplots()

hist = int(10*(1000/msec))
xhist = np.zeros((hist, ac._n_states))
uhist = np.zeros((hist, ac._n_inputs))
t = np.linspace(0, (hist-1)*dt, hist)
t = t - t[-1]
index = 0
lines = ax.plot(t, xhist)
ax.set_ylim(-10, 10)
ax.legend(['u', 'w', 'q', 'pitch', 'z'])

elevator = 0
thrust = 0

def press(event):
    global elevator
    global thrust
    if event.key == 'up':
        elevator -= np.deg2rad(1)
    if event.key == 'down':
        elevator += np.deg2rad(1)
    if event.key == 'q':
        thrust += 100
    if event.key == 'a':
        thrust -= 100
    print (elevator, thrust)


frame = 0
def tracker(self):
    global frame
    global index
    frame += 1
    ac.update(dt, [elevator, thrust])
    xhist[index] = ac._x
    #uhist[index] = ac._u
    index = (index + 1) % hist
    xplot = np.concatenate((xhist[index:], xhist[:index]))
    for i in range(len(lines)):
        if index == 3:
            lines[i].set_data(t, np.rad2deg(xplot[:, i]))
        else:
            lines[i].set_data(t, xplot[:, i])
    #return [ax.plot(t, xplot)[0]]
    #return [ax]


anim = FuncAnimation(fig, tracker, interval=msec, blit=False, repeat=False)
fig.canvas.mpl_connect('key_press_event', press)
plt.show()
