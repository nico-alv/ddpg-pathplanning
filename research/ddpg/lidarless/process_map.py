
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Button, Slider
import scipy.io
import time

file_path = 'Environment1.mat'
mat_data = scipy.io.loadmat(file_path)
BW = mat_data['BW']

fig, ax = plt.subplots()
plt.subplots_adjust(bottom=0.3)
cmap = plt.get_cmap('gray')
im = ax.imshow(BW, cmap=cmap, interpolation='none')

mouse_pressed = False
paint_mode = None
last_update_time = time.time()
update_interval = 0.05  # Actualizar cada 50 ms

history = []
brush_size = 3

def paint(event, mode):
    x, y = int(event.xdata), int(event.ydata)
    half_brush = brush_size // 2
    if 0 <= x < BW.shape[1] and 0 <= y < BW.shape[0]:
        if mode == 'paint':
            BW[max(0, y-half_brush):min(BW.shape[0], y+half_brush+1),
               max(0, x-half_brush):min(BW.shape[1], x+half_brush+1)] = 1
        elif mode == 'erase':
            BW[max(0, y-half_brush):min(BW.shape[0], y+half_brush+1),
               max(0, x-half_brush):min(BW.shape[1], x+half_brush+1)] = 0
        im.set_data(BW)
        plt.draw()

def on_press(event):
    global mouse_pressed, paint_mode
    if event.inaxes != ax:
        return
    mouse_pressed = True
    if event.button == 1:
        paint_mode = 'paint'
    elif event.button == 3:
        paint_mode = 'erase'
    history.append(BW.copy())
    paint(event, paint_mode)

def on_release(event):
    global mouse_pressed
    mouse_pressed = False

def on_motion(event):
    global last_update_time
    if mouse_pressed and event.inaxes == ax:
        current_time = time.time()
        if current_time - last_update_time > update_interval:
            paint(event, paint_mode)
            last_update_time = current_time

def undo(event):
    global BW
    if history:
        BW = history.pop()
        im.set_data(BW)
        plt.draw()

def update_brush_size(val):
    global brush_size
    brush_size = int(val)

fig.canvas.mpl_connect('button_press_event', on_press)
fig.canvas.mpl_connect('button_release_event', on_release)
fig.canvas.mpl_connect('motion_notify_event', on_motion)

def save(event):
    scipy.io.savemat('Modified_Environment1.mat', {'BW': BW})
    print("Mapa guardado en 'Modified_Environment1.mat'")

ax_save = plt.axes([0.8, 0.15, 0.1, 0.075])
btn_save = Button(ax_save, 'Guardar')
btn_save.on_clicked(save)

ax_undo = plt.axes([0.7, 0.15, 0.1, 0.075])
btn_undo = Button(ax_undo, 'Ctrl+Z')
btn_undo.on_clicked(undo)

ax_slider = plt.axes([0.2, 0.1, 0.65, 0.03], facecolor='lightgoldenrodyellow')
slider = Slider(ax_slider, 'Brush Size', 1, 10, valinit=brush_size, valstep=1)
slider.on_changed(update_brush_size)

plt.show()
