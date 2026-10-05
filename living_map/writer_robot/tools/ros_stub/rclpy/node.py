"""Node stand-in: an in-process topic bus + harness-driven clock and timers."""
from .time import Time

BUS = {}            # topic -> list of callbacks
PUBLISHED = {}      # topic -> list of messages (every publish, for tests)
TIMERS = []         # [period, next_due, callback]
PARAM_OVERRIDES = {}
CLOCK = {'t': 0.0}
LOG = []


class _Param:
    def __init__(self, value):
        self.value = value


class _Publisher:
    def __init__(self, topic):
        self.topic = topic

    def publish(self, msg):
        PUBLISHED.setdefault(self.topic, []).append(msg)
        for cb in list(BUS.get(self.topic, [])):
            cb(msg)


class _Clock:
    def now(self):
        return Time(seconds=CLOCK['t'])


class _Logger:
    def __init__(self, name):
        self.name = name

    def _log(self, lvl, msg, **kw):
        LOG.append((CLOCK['t'], self.name, lvl, str(msg)))

    def debug(self, msg, **kw):
        self._log('DEBUG', msg)

    def info(self, msg, **kw):
        self._log('INFO', msg)

    def warning(self, msg, **kw):
        self._log('WARN', msg)

    warn = warning

    def error(self, msg, **kw):
        self._log('ERROR', msg)


class Node:
    def __init__(self, name):
        self._name = name
        self._params = {}

    def declare_parameter(self, name, value=None, descriptor=None):
        v = PARAM_OVERRIDES.get(name, value)
        self._params[name] = _Param(v)
        return self._params[name]

    def get_parameter(self, name):
        return self._params[name]

    def create_subscription(self, msg_type, topic, cb, qos):
        BUS.setdefault(topic, []).append(cb)

    def create_publisher(self, msg_type, topic, qos):
        return _Publisher(topic)

    def create_timer(self, period, cb):
        TIMERS.append([period, CLOCK['t'] + period, cb])

    def get_clock(self):
        return _Clock()

    def get_logger(self):
        return _Logger(self._name)

    def destroy_node(self):
        pass


def reset():
    BUS.clear()
    PUBLISHED.clear()
    TIMERS.clear()
    PARAM_OVERRIDES.clear()
    LOG.clear()
    CLOCK['t'] = 0.0


def run_timers(now):
    """Advance the clock to `now` and fire every due timer."""
    CLOCK['t'] = now
    for tm in TIMERS:
        while tm[1] <= now + 1e-9:
            tm[1] += tm[0]
            tm[2]()
