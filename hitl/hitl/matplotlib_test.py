from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

# Check if 3D plotting works
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter([1, 2, 3], [4, 5, 6], [7, 8, 9])

plt.show()
