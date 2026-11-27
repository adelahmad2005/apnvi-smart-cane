"""Read LaserScan sweeps (and /odom poses if recorded) from a ROS 2 bag folder (sqlite3 .db3),
without ROS. Messages are stored as CDR: a 4-byte header, then the fields, each aligned to its size."""
import glob, os, sqlite3, struct
import numpy as np


class Cdr:
    def __init__(self, data):
        self.d, self.o = data, 4
        self.le = '<' if data[1] == 1 else '>'

    def align(self, n):
        self.o += (-(self.o - 4)) % n

    def get(self, fmt):
        n = struct.calcsize(fmt)
        self.align(n)
        v = struct.unpack_from(self.le + fmt, self.d, self.o)[0]
        self.o += n
        return v

    def string(self):
        n = self.get('I')
        s = self.d[self.o:self.o + n - 1].decode('utf8', 'replace')
        self.o += n
        return s

    def floats(self, kind='f'):
        n = self.get('I')
        size = 4 if kind == 'f' else 8
        self.align(size)
        a = np.frombuffer(self.d, dtype=self.le + kind + str(size), count=n, offset=self.o)
        self.o += n * size
        return a


def parse_scan(data):
    c = Cdr(data)
    sec, nsec = c.get('i'), c.get('I')
    frame = c.string()
    amin, amax, ainc, tinc, stime, rmin, rmax = [c.get('f') for _ in range(7)]
    ranges = c.floats().astype(np.float32)
    inten = c.floats().astype(np.float32)
    return dict(t=sec + nsec * 1e-9, frame=frame, angle_min=amin, angle_max=amax, angle_inc=ainc,
                time_inc=tinc, scan_time=stime, range_min=rmin, range_max=rmax, ranges=ranges,
                intensities=inten)


def parse_odom(data):
    c = Cdr(data)
    sec, nsec = c.get('i'), c.get('I')
    c.string(); c.string()
    px, py, pz, qx, qy, qz, qw = [c.get('d') for _ in range(7)]
    lost = (qx == 0 and qy == 0 and qz == 0 and qw == 0)
    yaw = np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
    return (sec + nsec * 1e-9, px, py, yaw, float(lost))


def read_bag(folder, scan_topic='/scan_raw', odom_topic='/odom'):
    scans, odom, rec_t = [], [], []
    for db in sorted(glob.glob(os.path.join(folder, '*.db3'))):
        con = sqlite3.connect(db)
        topics = {name: (tid, typ) for tid, name, typ in con.execute('select id, name, type from topics')}
        if scan_topic in topics:
            for ts, data in con.execute('select timestamp, data from messages where topic_id=? order by timestamp',
                                        (topics[scan_topic][0],)):
                s = parse_scan(bytes(data)); s['rec_t'] = ts * 1e-9
                scans.append(s)
        if odom_topic in topics:
            for ts, data in con.execute('select timestamp, data from messages where topic_id=? order by timestamp',
                                        (topics[odom_topic][0],)):
                odom.append(parse_odom(bytes(data)))
        con.close()
    return scans, np.array(odom), topics
