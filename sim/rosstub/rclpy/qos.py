class _E:
    TRANSIENT_LOCAL = 1; VOLATILE = 0; RELIABLE = 1; BEST_EFFORT = 0; KEEP_LAST = 1
DurabilityPolicy = ReliabilityPolicy = HistoryPolicy = _E
class QoSProfile:
    def __init__(self, **kw): self.__dict__.update(kw)
qos_profile_sensor_data = QoSProfile(depth=5, history=1, reliability=0, durability=0)
