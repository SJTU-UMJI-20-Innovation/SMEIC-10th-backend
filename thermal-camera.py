import seeed_mlx9064x
import json
import sys
camera = seeed_mlx9064x.grove_mxl90641()
camera.refresh_rate = seeed_mlx9064x.RefreshRate.REFRESH_8_HZ
frame = [0] * 192
camera.getFrame(frame) # Flush the first garbage frame
while True:
    try:
        camera.getFrame(frame)
    except ValueError:
        continue
    print(json.dumps({"data": frame}))
    sys.stdout.flush() # and the stdout 'data' event is triggered
