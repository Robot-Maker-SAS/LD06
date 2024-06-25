import socket
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

UDP_IP = "127.0.0.1"  # Використання localhost
UDP_PORT = 12348  # Зміна на новий порт

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})

# Ініціалізація даних
angles = np.linspace(0, 2 * np.pi, 360, endpoint=False)
distances = np.zeros(360)

line, = ax.plot(angles, distances)

def update(data):
    global distances
    distances = np.frombuffer(data, dtype=np.uint32)
    line.set_ydata(distances)
    return line,

def data_gen():
    while True:
        data, _ = sock.recvfrom(1440)  # 360 значень по 4 байти кожне (360 * 4 = 1440)
        yield data

ani = animation.FuncAnimation(fig, update, data_gen, blit=True, cache_frame_data=False)
plt.show()
