import matplotlib.pyplot as plt
import matplotlib.image as mpimg

# Map settings — adjust these to match your map
map_width = 67     # in grid cells
map_height = 85    # in grid cells
img_path = "/tmp/inflated_map_debug.png"  # path to your map image

# Load map image
img = mpimg.imread(img_path)
img_height, img_width = img.shape[:2]

def onclick(event):
    if event.xdata is not None and event.ydata is not None:
        pixel_x = int(event.xdata)
        pixel_y = int(event.ydata)

        # Convert from image pixels to grid coords
        grid_x = int((pixel_x / img_width) * map_width)
        grid_y = map_height - int((pixel_y / img_height) * map_height)

        print(f"Clicked pixel: ({pixel_x}, {pixel_y}) → Grid cell: ({grid_x}, {grid_y})")

# Plot and activate click
fig, ax = plt.subplots()
ax.imshow(img, cmap='gray', origin='upper')
ax.set_title("Click on the map to get grid coordinates")
cid = fig.canvas.mpl_connect('button_press_event', onclick)
plt.show()
