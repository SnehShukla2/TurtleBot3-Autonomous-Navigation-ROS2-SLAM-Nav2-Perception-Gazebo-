import matplotlib.pyplot as plt
import matplotlib.image as mpimg

# Load the image
img_path = '/tmp/inflated_map_debug.png'
img = mpimg.imread(img_path)

# Convert cell coordinates to pixel space
# Map size from your occupancy grid (67 x 85)
map_width = 67
map_height = 85

# Image dimensions
img_height, img_width = img.shape[0], img.shape[1]

# Grid cell to mark
cells_to_mark = [(20, 27), (61, 23)]  # (x, y) format

# Convert cell coords to image pixel coords
def grid_to_pixel(cell):
    gx, gy = cell
    px = int((gx / map_width) * img_width)
    py = img_height - int((gy / map_height) * img_height)  # Y is flipped!
    return (px, py)

# Plot image and markers
plt.figure(figsize=(10, 10))
plt.imshow(img, cmap='gray')
plt.title("Map with Cell Markers")

# Draw circles
for cell in cells_to_mark:
    px, py = grid_to_pixel(cell)
    plt.plot(px, py, 'go', markersize=10)  # Green circles

# Optional: annotate the point
    plt.text(px + 5, py - 5, f"{cell}", color='lime', fontsize=10)

plt.xlabel("X (pixels)")
plt.ylabel("Y (pixels)")
plt.grid(True)
plt.show()
