# Where the movement tracker alone ended (raw pose of the last map node vs the first).
import sqlite3, sys, struct, math
for f in sys.argv[1:]:
    rows = sqlite3.connect(f).execute("SELECT id, pose FROM Node ORDER BY id").fetchall()
    def p(b):
        m = struct.unpack("12f", b[:48]); return m[3], m[7], math.atan2(m[4], m[0])
    a, b = p(rows[0][1]), p(rows[-1][1])
    print("%s odom end %.2f m, %.1f deg (%d nodes)" % (f.split("/")[-2], math.hypot(b[0]-a[0], b[1]-a[1]), math.degrees((b[2]-a[2]+math.pi)%(2*math.pi)-math.pi), len(rows)))
