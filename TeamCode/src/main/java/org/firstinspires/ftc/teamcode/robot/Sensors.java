package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class Sensors {
    public AprilTagProcessor aprilTag;

    public VisionPortal visionPortal;
    public Sensors(HardwareMap hardwareMap) {
        // Using Webcam
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class,"Webcam 1")
        );
    }
    public void updateRobot() {

    }
}
