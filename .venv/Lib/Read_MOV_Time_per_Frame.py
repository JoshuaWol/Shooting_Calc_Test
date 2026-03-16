import av
import numpy as np
from pathlib import Path
import os

def frame_times_pts(path, start_frame=0, num_frames=150, stream_index=0):
    """
    Returns:
      frame_idxs: absolute frame indices (decoded order)
      times: presentation times in seconds (float)
      deltas: seconds between consecutive frame presentation times
    """


    container = av.open(path)
    stream = container.streams.video[stream_index]

    times = []
    frame_idxs = []

    decoded_i = -1
    for frame in container.decode(stream):
        decoded_i += 1
        if decoded_i < start_frame:
            continue
        if len(times) >= num_frames:
            break

        # Best effort timestamp (seconds). Handles VFR well.
        t = frame.time  # float seconds or None
        if t is None:
            print("No Frame Time")
            # Fallback if needed: use PTS * time_base
            if frame.pts is not None and stream.time_base is not None:
                t = float(frame.pts * stream.time_base)
            else:
                raise RuntimeError("Frame has no timestamp (PTS/time).")

        frame_idxs.append(decoded_i)
        times.append(float(t))

    container.close()

    times = np.array(times, dtype=float)
    if len(times) < 2:
        deltas = np.array([], dtype=float)
    else:
        deltas = np.diff(times)

    return frame_idxs, times, deltas

directory = Path(__file__).parent
path_mov = os.path.join(directory, "IMG_3053.mov")
frame_idxs, times, deltas = frame_times_pts(path_mov, start_frame=1000, num_frames=240)

print("First 10 frame times (s):", times[:10])
print("First 10 deltas (ms):", deltas[:10] * 1000)
if len(deltas):
    print(f"Mean delta: {deltas.mean()*1000:.3f} ms")
    print(f"Min/Max delta: {deltas.min()*1000:.3f}/{deltas.max()*1000:.3f} ms")