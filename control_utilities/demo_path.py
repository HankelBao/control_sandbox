import sys

from control_utilities.path import RandomPathGenerator, Path

def plot(generator, seed=1.0):
    import matplotlib.pyplot as plt
    from matplotlib.widgets import Slider

    plot_ax = plt.axes([0.1, 0.2, 0.8, 0.75])
    seed_axes = plt.axes([0.1, 0.05, 0.8, 0.05])
    seed_slider = Slider(
        seed_axes, "Seed", 1, 100, valinit=int(seed), valstep=1
    )
    plt.sca(plot_ax)

    def update(val):
        path = Path(generator.generatePath(seed=seed_slider.val))
        plt.cla()

        path.plot(color='-r', show=False)

    update(seed)

    seed_slider.on_changed(update)
    plt.show()


if __name__ == "__main__":
    import matplotlib.pyplot as plt

    if len(sys.argv) == 2:
        seed = int(sys.argv[1])
    else:
        seed = 1.0

    generator = RandomPathGenerator(x_max=100, y_max=100)
    plot(generator, seed)
