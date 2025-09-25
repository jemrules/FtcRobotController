package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

public class Movement {
    public VectorF position;
    public double targetHeading;
    public IMU imu;
    public DcMotor left_motor;
    public DcMotor right_motor;
    public Movement(VectorF default_position, double default_heading, HardwareMap hardwareMap) {
        // Set the default position and default heading
        position=default_position;
        targetHeading=default_heading;

        // CONTROL HUB ORIENTATION SETTINGS
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection,usbDirection);

        imu=hardwareMap.get(IMU.class,"imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // Set DC Motor variables
        left_motor=hardwareMap.get(DcMotor.class,"left_motor");
        right_motor=hardwareMap.get(DcMotor.class,"right_motor");

        // MOTOR DIRECTION SETTINGS
        left_motor.setDirection(DcMotorSimple.Direction.FORWARD);
        right_motor.setDirection(DcMotorSimple.Direction.FORWARD);
    }
    // Set default_heading to 0.0 if default_heading isn't given
    public Movement(VectorF default_position, HardwareMap hardwareMap) {this(default_position,0.0,hardwareMap);}
    public void RobotStart() {
        imu.resetYaw();
    } // Reset Yaw to 0
    // Get heading from the IMU (Control Hub)
    public double getHeading() {
        YawPitchRollAngles YPRAimu = imu.getRobotYawPitchRollAngles();
        return YPRAimu.getYaw(AngleUnit.RADIANS);
    }
    public void setHeading(double target_heading) {

    }
    public void lookAt(VectorF target_position) {

    }
    // Drive in that direction
    public void driveTo(VectorF target_vector,boolean drive_and_turn) {

    }
    public void driveTo(double orientation,double distance,boolean drive_and_turn) {
        driveTo(new VectorF(
                (float)(cos(orientation)*distance),
                (float)(sin(orientation)*distance)),
                drive_and_turn);
    }
}
