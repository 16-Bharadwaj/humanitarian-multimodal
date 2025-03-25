import gurobipy as gp
from gurobipy import GRB

import signal


# Global flag to handle interruption
interrupted = False


def signal_handler(sig, frame):
    global interrupted
    interrupted = True
    print("\nInterrupt received! Stopping optimization...")


# Register the signal handler for KeyboardInterrupt (Ctrl+C)
signal.signal(signal.SIGINT, signal_handler)


def my_callback(model, where):
    global interrupted
    if where == GRB.Callback.MIPNODE:
        if interrupted:
            model.terminate()  # Stop optimization safely


def model():
    pass


if __name__ == '__main__':
    pass