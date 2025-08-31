from invp_pendulum import Pendulum

pendulum = None


def runGame(screen, refresh):
    global pendulum
    if pendulum is None:
        pendulum = Pendulum(screen, refresh)
    pendulum.render()