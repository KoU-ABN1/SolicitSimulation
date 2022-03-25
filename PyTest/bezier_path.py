import math
import matplotlib.pyplot as plt
import numpy as np
import time

plt.figure(figsize=(12, 6))
plt.xlim(-10, 2)
plt.ylim(0, 6)
plt.grid()

def calc_bezier_path(x1, y1, yaw1, x2, y2, yaw2):
    a1 = math.sin(yaw1)
    b1 = -math.cos(yaw1)
    c1 = y1 * math.cos(yaw1) - x1 * math.sin(yaw1)
    a2 = math.sin(yaw2)
    b2 = -math.cos(yaw2)
    c2 = y2 * math.cos(yaw2) - x2 * math.sin(yaw2)

    l = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    d1 = abs(a1 * x2 + b1 * y2 + c1)
    d2 = abs(a2 * x1 + b2 * y1 + c2)
    
    k1 = 0.5 * math.sqrt(l**2 - d1**2)
    k2 = 0.5 * math.sqrt(l**2 - d2**2)

    p1 = [x1, y1]
    p2 = [x1 + k1 * math.cos(yaw1), y1 + k1 * math.sin(yaw1)]
    p3 = [x2 - k2 * math.cos(yaw2), y2 - k2 * math.sin(yaw2)]
    p4 = [x2, y2]

    plt.plot(p2[0], p2[1], 'o')
    plt.plot(p3[0], p3[1], 'o')

    xs = []
    ys = []
    ts = np.arange(0, 1, 1 / 20)
    for t in ts:
        x = p1[0] * (1-t)**3 + 3 * p2[0] * (1-t)**2 * t + 3 * p3[0] * (1-t) * t**2 + p4[0] * t**3
        y = p1[1] * (1-t)**3 + 3 * p2[1] * (1-t)**2 * t + 3 * p3[1] * (1-t) * t**2 + p4[1] * t**3
        xs.append(x)
        ys.append(y)
    
    return [xs, ys]


if __name__ == '__main__': 
    t0 = time.time()

    robot_x = 0
    robot_y = 0
    robot_yaw = 90 / 180.0 * math.pi

    target_x = 2
    target_y = 6
    target_yaw = 90 / 180.0 * math.pi

    path = calc_bezier_path(robot_x, robot_y, robot_yaw, target_x, target_y, target_yaw)

    print(time.time() - t0)

    arrow_length = 0.5
    plt.arrow(robot_x,
              robot_y,
              arrow_length * math.cos(robot_yaw),
              arrow_length * math.sin(robot_yaw),
              width=0.05,
              fc="black",
              ec="black")
    plt.arrow(target_x,
              target_y,
              arrow_length * math.cos(target_yaw),
              arrow_length * math.sin(target_yaw),
              width=0.05,
              fc="black",
              ec="black")

    plt.plot(path[0], path[1], 'o', c='red')

    plt.show()
