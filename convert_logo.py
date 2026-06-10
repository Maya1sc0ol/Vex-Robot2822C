from PIL import Image
import sys

if len(sys.argv) < 2:
    print("Usage: python convert_logo.py <path_to_logo.png>")
    sys.exit(1)

img = Image.open(sys.argv[1]).convert("RGBA")
img = img.resize((100, 100), Image.LANCZOS)
w, h = img.size

print("#pragma once")
print(f"#define LOGO_WIDTH  {w}")
print(f"#define LOGO_HEIGHT {h}")
print(f"static const uint32_t logo_pixels[{h}][{w}] = {{")
for y in range(h):
    row = []
    for x in range(w):
        r, g, b, a = img.getpixel((x, y))
        if a < 128:
            row.append("0x00FFFFFF")
        else:
            row.append(f"0x{r:02X}{g:02X}{b:02X}")
    print("    {" + ", ".join(row) + "},")
print("};")
