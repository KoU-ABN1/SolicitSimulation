import math
import matplotlib.pyplot as plt
import numpy as np
import time


plt.figure(figsize=(12, 5))
plt.xlim(-10, 2)
plt.ylim(-1, 4)
plt.grid()


def rectify_angle(angle):
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle <= -math.pi:
        angle += 2 * math.pi
    return angle


def calc_cost(sx, sy, syaw, ex, ey, eyaw, T, alpha):
    te = math.atan2(ey - sy, ex - sx)

    angular_diff = abs(rectify_angle(syaw - te)) + abs(
        rectify_angle(eyaw - te))
    linear_diff = math.sqrt((ex - sx)**2 + (ey - sy)**2)

    w = [1, 1, 1, 0.5, 2.5]
    c = [0, 0, 0, 0, 0]
    c[0] = linear_diff
    c[1] = angular_diff
    c[2] = angular_diff / linear_diff
    c[3] = T
    c[4] = abs(alpha)

    cost = 0
    for i in range(len(c)):
        cost += w[i] * c[i]
    return cost


def calc_optimal_target_pose(rx, ry, ryaw, rv, px, py, pyaw, pv, fov, r, s1,
                             s2):

    dist_interval = 0.1

    min_cost = 100000000
    target_pose = []

    for i in range(s1):
        person_pos = [
            px + i * dist_interval * math.cos(pyaw),
            py + i * dist_interval * math.sin(pyaw)
        ]

        t2 = i * dist_interval / pv

        for j in range(s2):
            theta = (pyaw - fov / 2) + j * (fov / (s2 - 1))
            candidate = [
                person_pos[0] + r * math.cos(theta),
                person_pos[1] + r * math.sin(theta),
                rectify_angle(theta + math.pi)
            ]

            t1 = 1.3 * math.sqrt((rx - candidate[0])**2 +
                                 (ry - candidate[1])**2) / rv

            if t1 < t2:
                #plt.scatter(candidate[0], candidate[1], c='green', s=1)

                cost = calc_cost(rx, ry, ryaw, candidate[0], candidate[1],
                                 candidate[2], t2, theta - pyaw)

                if cost < min_cost:
                    min_cost = cost
                    target_pose = candidate

    return target_pose


if __name__ == '__main__':
    t0 = time.time()

    robot_x = 0
    robot_y = 0
    robot_yaw = 180 / 180.0 * math.pi
    robot_v = 0.7

    person_x = -6
    person_y = 0.2
    person_yaw = 0 / 180.0 * math.pi
    person_v = 0.80

    person_fov_theta = 120 / 180.0 * math.pi
    person_fov_length = 1.5

    sample_times_1 = 60
    sample_times_2 = 20

    target_pose = calc_optimal_target_pose(robot_x, robot_y, robot_yaw,
                                           robot_v, person_x, person_y,
                                           person_yaw, person_v,
                                           person_fov_theta, person_fov_length,
                                           sample_times_1, sample_times_2)

    print(time.time() - t0)

    print(target_pose)
    print(rectify_angle(math.pi + person_yaw - target_pose[2]) / math.pi * 180)

    arrow_length = 0.5
    plt.arrow(robot_x,
              robot_y,
              arrow_length * math.cos(robot_yaw),
              arrow_length * math.sin(robot_yaw),
              width=0.05,
              fc="black",
              ec="black")
    plt.arrow(person_x,
              person_y,
              arrow_length * math.cos(person_yaw),
              arrow_length * math.sin(person_yaw),
              width=0.05,
              fc="black",
              ec="black")
    plt.arrow(target_pose[0],
              target_pose[1],
              arrow_length * math.cos(target_pose[2]),
              arrow_length * math.sin(target_pose[2]),
              width=0.05,
              fc="red",
              ec="red")

    line_length = 10
    plt.plot([person_x, person_x + line_length * math.cos(person_yaw)],
             [person_y, person_y + line_length * math.sin(person_yaw)],
             c="grey")

    plt.plot(target_pose[0], target_pose[1], 'o', c='red')

    plt.show()
