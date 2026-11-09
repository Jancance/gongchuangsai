R_mm, R_px = 150.0, 618.0869533049624
k = R_mm / R_px
print('radius_px=%.1f  mm_per_px=%.5f  px_per_mm=%.3f' % (R_px, k, 1 / k))
cx, cy = 982.6, 394.3
print('disc center px (%.1f, %.1f) -> offset from image center (960,540): (%+.1f, %+.1f) px = (%+.1f, %+.1f) mm'
      % (cx, cy, cx - 960, cy - 540, (cx - 960) * k, (cy - 540) * k))
print('disc top edge in image: y=%.0f (%.0f px above frame)' % (cy - R_px, -(cy - R_px)))
print()
regs = {'left': (301, 1, 614, 327), 'right': (1102, 3, 639, 334), 'down': (676, 648, 578, 409)}
for n, (x, y, w, h) in regs.items():
    px, py = x + w / 2.0, y + h / 2.0
    print('%-6s center=(%6.1f,%6.1f)  d_px=(%+7.1f,%+7.1f)  d_mm=(%+7.1f,%+7.1f)  dist=%.1fmm  box=%dx%dmm'
          % (n, px, py, px - 960, py - 540, (px - 960) * k, (py - 540) * k,
             ((px - 960) ** 2 + (py - 540) ** 2) ** 0.5 * k, w * k, h * k))
