"""Renderiza capturas do teste C: executar apos make test (requer Pillow)."""
from pathlib import Path
from PIL import Image, ImageDraw

root = Path(__file__).resolve().parents[1]
names = ["level", "bank", "inverted", "up", "down", "rates"]
labels = ["Nivelado", "Curva: Roll 35 / Pitch 15", "Invertido: Roll 180",
          "Nariz +90: VERT", "Nariz -90: VERT", "Velocidades XYZ"]
canvas = Image.new("RGB", (1056, 888), (18, 23, 30))
draw = ImageDraw.Draw(canvas)
for i, (name, label) in enumerate(zip(names, labels)):
    data = (root / f"build/panel-{name}.bin").read_bytes()
    assert len(data) == 1024
    im = Image.new("RGB", (128, 64), (0, 0, 0))
    for y in range(64):
        for x in range(128):
            if data[x + (y // 8) * 128] & (1 << (y % 8)):
                im.putpixel((x, y), (205, 240, 255))
    im.resize((768, 384), Image.Resampling.NEAREST).save(root / f"build/panel-{name}.png")
    x, y = 16 + (i % 2) * 528, 12 + (i // 2) * 296
    draw.text((x, y), label, fill=(230, 235, 245))
    canvas.paste(im.resize((512, 256), Image.Resampling.NEAREST), (x, y + 22))
canvas.save(root / "Docs/Preview_Painel_Voo.png")
