package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

import java.util.Vector;

import static java.lang.Math.*;

public class Movement {
    public VectorF position;
    public double targetHeading;
    public IMU imu;
    public Movement(VectorF default_position,double default_heading) {
        // Set the default position and default heading
        position=default_position;
        targetHeading=default_heading;

        // CONTROL HUB ORIENTATION SETTINGS
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection,usbDirection);

        imu=hardwareMap.get(IMU.class,"imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }
    // Set default_heading to 0.0 if default_heading isn't given
    public Movement(VectorF default_position) {this(default_position,0.0);}
    public void RobotStart() {
        imu.resetYaw();
    }
    // Get heading from the IMU (Control Hub)
    public double getHeading() {
        return 0.0;
    }
    public void setHeading(double target_heading) {

    }
    public void lookAt(VectorF target_position) {

    }
    // Drive in that direction
    public void driveTo(VectorF target_vector) {

    }
    public void driveTo(double orientation,double distance) {
        driveTo(new VectorF(
                (float)(cos(orientation)*distance),
                (float)(sin(orientation)*distance)));
    }
}
