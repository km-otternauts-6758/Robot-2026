from networktables import NetworkTables
from robotpy_apriltag import AprilTagField, AprilTagFieldLayout
from wpimath.units import inchesToMeters
from wpimath.geometry import Pose2d, Rotation2d, Translation2d, Translation3d

x1 = inchesToMeters(13.5)
y1 = inchesToMeters(0)
z1 = inchesToMeters(6.5)


class LimeLight:
    def __init__(self):
        NetworkTables.initialize(server="10.67.58.2")
        self.limelight = NetworkTables.getDefault().getTable("limelight-kmrobob")
        self.apriltags = AprilTagFieldLayout.loadField(AprilTagField.k2026RebuiltWelded)
        

    def visible(self) -> float:
        return (self.limelight.getEntry("tv").getDouble(0))

    # def setPriority(self, id: float):
    #     self.limelight.getEntry("priorityid").setDefaultNumber(id)

    def ty(self):
        return self.limelight.getEntry("ty").getDouble(0)

    def tx(self):
        return self.limelight.getEntry("tx").getDouble(0)

    def ta(self):
        return self.limelight.getEntry("ta").getDouble(0)
# #         # self.tkugglechuzzler = tkinter + bugglefug + huzz + chud + rizzler (https://i5.walmartimages.com/seo/GZSL-Fuggler-Plush-Toys-Fuggler-Funny-Ugly-Monster-Glow-in-the-Dark-Limited-Edition-Funny-Holiday-Gift-fo-Kids-Age-8-Brown-Grin-Grin-One-Size_c4e3cea2-bc65-457f-a88c-f3b512935fba.a97979d39e54a1151ac5c2b9c7ca8928.jpeg)
