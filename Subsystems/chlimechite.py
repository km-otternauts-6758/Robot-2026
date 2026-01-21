import wpilib
from networktables import NetworkTables
import robotpy_apriltag


class LimeLight:
    def __init__(self):
        NetworkTables.initialize(server="10.67.58.2")
        self.limelight = NetworkTables.getDefault().getTable("limelight-kmrobot")
        self.apriltags = robotpy_apriltag

    def visible(self, key: str, defaultValue: float):
        return self.limelight.getNumber(key, defaultValue)

    def setPriority(self, id: float):
        self.limelight.getEntry("priorityid").setDefaultNumber(id)

    def values(self):
        return self.limelight.getEntry("tx").getDouble(0), self.limelight.getEntry(
            "ty"
        ).getDouble(0)


# #         # self.tkugglechuzzler = tkinter + bugglefug + huzz + chud + rizzler (https://i5.walmartimages.com/seo/GZSL-Fuggler-Plush-Toys-Fuggler-Funny-Ugly-Monster-Glow-in-the-Dark-Limited-Edition-Funny-Holiday-Gift-fo-Kids-Age-8-Brown-Grin-Grin-One-Size_c4e3cea2-bc65-457f-a88c-f3b512935fba.a97979d39e54a1151ac5c2b9c7ca8928.jpeg)
