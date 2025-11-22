import matplotlib.pyplot as plt
import matplotlib.image as mpimg

img = mpimg.imread('/tmp/inflated_map_debug.png')  # or astar_search_debug.png

plt.figure(figsize=(10, 10))
plt.imshow(img, cmap='gray', origin='lower')
plt.title("Inflated Map with Axes")
plt.xlabel("X (columns)")
plt.ylabel("Y (rows)")
plt.grid(True)
plt.show()
