import serial
import threading
import numpy as np
import matplotlib.pyplot as plt

# -----------------------------
# Serial config
# -----------------------------
PORT = "COM4"
BAUD = 115200
ser = serial.Serial(PORT, BAUD, timeout=0.0)

Q30_SCALE = 2**30

def q30_to_float(q):
    return q / Q30_SCALE

def quat_rotate(q, v):
    w, x, y, z = q
    qv = np.array([x, y, z])
    t = 2 * np.cross(qv, v)
    return v + w * t + np.cross(qv, t)


# -----------------------------
# Box geometry (local frame)
# -----------------------------

BOX_SIZE = np.array([0.4, 1.0, 0.25])  # length, width, height
hx, hy, hz = BOX_SIZE / 2

BOX_VERTS = np.array([
    [-hx, -hy, -hz],
    [ hx, -hy, -hz],
    [ hx,  hy, -hz],
    [-hx,  hy, -hz],
    [-hx, -hy,  hz],
    [ hx, -hy,  hz],
    [ hx,  hy,  hz],
    [-hx,  hy,  hz],
])

EDGES = [
    (0,1),(1,2),(2,3),(3,0),
    (4,5),(5,6),(6,7),(7,4),
    (0,4),(1,5),(2,6),(3,7),
]

# -----------------------------
# Shared state
# -----------------------------
latest_q = np.array([1.0, 0.0, 0.0, 0.0])
lock = threading.Lock()
stop = False

# -----------------------------
# Serial reader thread
# -----------------------------
def reader():
    global latest_q
    buf = bytearray()

    while not stop:
        buf.extend(ser.read(256))
        while b"\n" in buf:
            line, _, buf = buf.partition(b"\n")
            try:
                q1, q2, q3, q0 = map(int, line.split())
            except ValueError:
                continue

            q = np.array([
                q30_to_float(q0),
                q30_to_float(q1),
                q30_to_float(q2),
                q30_to_float(q3),
            ], float)

            n = np.linalg.norm(q)
            if n == 0:
                continue
            q /= n

            with lock:
                latest_q = q

threading.Thread(target=reader, daemon=True).start()

# -----------------------------
# Matplotlib setup
# -----------------------------
plt.ion()
fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")

ax.set_xlim([-1, 1])
ax.set_ylim([-1, 1])
ax.set_zlim([-1, 1])
ax.set_box_aspect([1,1,1])
ax.set_proj_type("ortho")
ax.set_axis_off()
ax.view_init(elev=20, azim=-160)
ax.set_title("Quaternion-Oriented Box")

GROUND_SIZE = 1.5
GRID_STEP = 0.3
Z0 = -0.6  # height of ground plane

xs = np.arange(-GROUND_SIZE, GROUND_SIZE + GRID_STEP, GRID_STEP)
ys = np.arange(-GROUND_SIZE, GROUND_SIZE + GRID_STEP, GRID_STEP)

for x in xs:
    ax.plot([x, x], [-GROUND_SIZE, GROUND_SIZE], [Z0, Z0],
            linewidth=0.8, alpha=0.3)

for y in ys:
    ax.plot([-GROUND_SIZE, GROUND_SIZE], [y, y], [Z0, Z0],
            linewidth=0.8, alpha=0.3)

# Create line artists (one per edge)
lines = []
for _ in EDGES:
    (line,) = ax.plot([0,0],[0,0],[0,0], lw=2)
    lines.append(line)

def on_close(event):
    global stop
    stop = True
    ser.close()

fig.canvas.mpl_connect("close_event", on_close)

# -----------------------------
# Update callback (fast)
# -----------------------------
def update():
    with lock:
        q = latest_q.copy()

    print(q)
    # Rotate vertices
    verts = np.array([quat_rotate(q, v) for v in BOX_VERTS])

    # Update edges
    for line, (i, j) in zip(lines, EDGES):
        xs = [verts[i,0], verts[j,0]]
        ys = [verts[i,1], verts[j,1]]
        zs = [verts[i,2], verts[j,2]]
        line.set_data(xs, ys)
        line.set_3d_properties(zs)

    fig.canvas.draw_idle()
    fig.canvas.flush_events()

# High-frequency timer
timer = fig.canvas.new_timer(interval=5)  # ~200 Hz
timer.add_callback(update)
timer.start()

plt.show(block=True)
