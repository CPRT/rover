class JointData:
    def __init__(self, name, ax):
        self.name = name
        self.time = 0
        self.target = 0
        self.state = 0
        self.times = []
        self.states = []
        self.targetsHistory = []
        self.ax = ax
        self.ax.set_xlabel("Time (s)")
        self.ax.set_ylabel("State")
        self.ax.set_title(name)
        self.ax.legend(["State", "Target"])
        self.ax.grid()
        self.line1 = self.ax.plot([], [], "-", label="State")[0]
        self.line2 = self.ax.plot([], [], "--", label="Target")[0]

        seconds = 30  # Number of seconds to display on the graph

        self.MAX_POINTS = (
            seconds * 10
        ) - 50  # Maximum number of points to display on the graph (gotten from seconds)

        self.ax.legend()

    def plotGraph(self):
        self.targetsHistory.append(round(self.target, 2))
        self.times.append(round(self.time, 2))
        self.states.append(round(self.state, 2))

        if len(self.times) > self.MAX_POINTS:
            self.times.pop(0)
            self.states.pop(0)
            self.targetsHistory.pop(0)

        self.line1.set_xdata(self.times)
        self.line2.set_xdata(self.times)

        self.line1.set_ydata(self.states)
        self.line2.set_ydata(self.targetsHistory)

        self.ax.relim()
        self.ax.autoscale_view()

    def reset(self):
        self.times = []
        self.states = []
        self.targetsHistory = []
