from wpilib import SmartDashboard, Field2d
from Subsystems.thehappenings import shooterkP,shooterkI, shooterkD
from Subsystems.chlimechite import LimeLight
from utils.constants import kRPM



class PIDShizz:
    def __init__(self):
        self.lime = LimeLight()

        self.field = Field2d()
        SmartDashboard.putNumber("TY values", self.lime.ty())
        SmartDashboard.putData("Field", self.field)
        SmartDashboard.putNumber("Shooter P", shooterkP)
        SmartDashboard.putNumber("Shooter I", shooterkI)
        SmartDashboard.putNumber("Shooter D", shooterkD)
    def updater(self) -> None:
        return SmartDashboard.updateValues()
