package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

public class Sensors {
    public AprilTagProcessor aprilTag;
    public double yawToApril = 0.0;
    public VisionPortal visionPortal;
    public ArrayList<AprilTagDetection> currentDetections;
    public Sensors(HardwareMap hardwareMap) {
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        // Using Webcam
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class,"Webcam 1"),aprilTag);
        yawToApril=0.0;
        startStream();
    }
    public void update(Telemetry telemetry){
        detectTags();
        for (AprilTagDetection detection : currentDetections)  {

            // Original source data
            //double poseX = detection.rawPose.x;
            //double poseY = detection.rawPose.y;
            //double poseZ = detection.rawPose.z;

            yawToApril = detection.ftcPose.yaw;
            //double poseAY = rot.secondAngle;
            //double poseAZ = rot.thirdAngle;
        }
        telemetry.addData("Detections Number: ", currentDetections.size());
        telemetry.addData("Yaw: ", yawToApril);
        if(currentDetections.isEmpty()){
            yawToApril = 0.0;
        }
    }   
    public void detectTags() {
        currentDetections = aprilTag.getDetections();
    }
    public void startStream() {
        visionPortal.resumeStreaming();
    }
    public void stopStream() {
        visionPortal.stopStreaming();
    }

    public void telemetryOutput(Telemetry telemetry) {
        telemetry.addData("# Tags Detected",currentDetections.size());
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }
    }
}
