import numpy as np
from matplotlib import pyplot as plt

q = 0
v = 10000


q_hat = 0
v_hat = 0

dt = 1 / 10000


prev_q = 0

hist = []
v_hist = []
for i in range(1000):
    q += v * dt

    q_measured = round(q)
    q_hat += dt * v_hat

    if(q_measured != prev_q):
        q_err = q_measured - q_hat

        v_hat += 20.0 * q_err - 0.5 * v_hat

    prev_q = q_measured

    # print("q", q)
    # print("q_measured", q_measured)
    # print("q_hat", q_hat)

    hist.append([q, q_measured, q_hat])

    v_hist.append([v, v_hat])

    print()

plt.plot(hist)
plt.show()

plt.plot(v_hist)
plt.show()