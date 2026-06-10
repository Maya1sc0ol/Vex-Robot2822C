import re
import numpy as np
import matplotlib.pyplot as plt

with open("include/warbotTemplate/logoPrint/logo_data.hpp") as f:
    content = f.read()

vals = re.findall(r'0x([0-9A-Fa-f]+)', content)
data = np.array([int(v, 16) for v in vals], dtype=np.uint32).reshape(100, 100)

r = ((data >> 16) & 0xFF).astype(np.uint8)
g = ((data >> 8) & 0xFF).astype(np.uint8)
b = (data & 0xFF).astype(np.uint8)
img = np.stack([r, g, b], axis=-1)

plt.imshow(img)
plt.axis("off")
plt.show()
