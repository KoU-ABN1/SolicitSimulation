from unicodedata import ucd_3_2_0
import numpy as np
import math

v1 = 0.8
v2 = 1.0
k = 1.2
r = 2
t = 3  # vertical distance
s = -3 * math.tan(80 / 180.0 * math.pi)  # horizontal distance
print('s =', s)

alpha = 0 / 180.0 * math.pi

u = v1 * v1 / (k * k * v2 * v2)
a = 1 - u * u
b = 2 * r * math.cos(alpha) + 2 * s * u * u
c = r * r + t * t - u * u * s * s + 2 * r * t * math.sin(alpha)

delta = b * b - 4 * a * c
print('delta =', delta)

m1 = (-b - math.sqrt(delta)) / (2 * a)
m2 = (-b + math.sqrt(delta)) / (2 * a)
print('m =', m1, m2)

qx = m1 + r * math.cos(alpha)
qy = t + r * math.sin(alpha)
print('Q =', qx, qy)