# Count the kinds of links in an RTAB-Map file: neighbour steps, loop closures, proximity.
import sqlite3, sys
NAMES = {0: 'step', 1: 'loop', 2: 'near-space', 3: 'near-time', 4: 'user', 5: 'landmark'}
try:
    rows = sqlite3.connect(sys.argv[1]).execute(
        'SELECT type, count(*) FROM Link GROUP BY type').fetchall()
    print(' '.join('%s=%d' % (NAMES.get(t, t), c) for t, c in rows))
except Exception as error:
    print('no links (%s)' % error)
