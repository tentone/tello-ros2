from . import event

class Signal(object):
    All = event.Event('*')

signals = {}

def connect(receiver, sig=Signal.All):
    if sig in signals:
        receivers = signals[sig]
    else:
        receivers = signals[sig] = []
    receivers.append(receiver)

def disconnect(receiver, sig=Signal.All):
    if sig is Signal.All:
        for sig in signals:
            if receiver in signals[sig]:
                signals[sig].remove(receiver)
    elif sig in signals:
        if receiver in signals[sig]:
            signals[sig].remove(receiver)

def send(sig, **named):
    if sig in signals:
        receivers = signals[sig] + signals[Signal.All]
    else:
        receivers = signals[Signal.All]
    for receiver in receivers:
        receiver(event=sig, **named)
