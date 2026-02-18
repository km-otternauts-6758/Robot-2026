import wpilib





class SegmentTimeTracker:
    """
    Utilties for tracking how long certain chunks of code take
    including logging overall loop execution time
    """
    def __init__(self, longLoopThresh=0.53):
        self.longLoopThresh = longLoopThresh
        self.tracer = wpilib.Tracer()
        self.loopStartTime = wpilib.Timer.getFPGATimestamp()
        self.loopEndTime = wpilib.Timer.getFPGATimestamp()
        self.prevLoopStartTime = self.loopStartTime
        self.curPeriod = 0
        self.curLoopExecDur = 0

    def start(self):
        """
        Mark the start of a periodic loop
        """
        self.tracer.clearEpochs()
        self.prevLoopStartTime = self.loopStartTime
        self.loopStartTime = wpilib.Timer.getFPGATimestamp()

    def mark(self, name):
        """
        Mark an intermdeate step complete during a periodic loop
        """
        self.tracer.addEpoch(name)

    def end(self):
        """
        Mark the end of a periodic loop
        """
        self.loopEndTime = wpilib.Timer.getFPGATimestamp()
        self.curPeriod = self.loopStartTime - self.prevLoopStartTime
        self.curLoopExecDur = self.loopEndTime - self.loopStartTime
        if self.curLoopExecDur > self.longLoopThresh:
            self.tracer.printEpochs()

