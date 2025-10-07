package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;

public class Movement {
    // SETTINGS
    public static double TURN_SCALE=1;
    public static double DRIVE_SCALE=1;
    public static double MOTOR_RPM=6000.0; // The Motors RPM
    public static double GEAR_RATIO=1.0/20.0; // The gear ratio coming out of the motor
    public static double WHEEL_DIAMETER=92.0/1000.0; // The diameter of the wheels [mm] to [m]
    public static double WHEEL_SPACING=38.0/100.0; // The distance between the left and right wheels [cm] to [m]



    public VectorF position;
    public double turn_rate;
    public VectorF movement_vector;
    public IMU imu;
    public DcMotor left_motor;
    public DcMotor right_motor;
    public Movement(VectorF default_position, double default_heading, HardwareMap hardwareMap) {
        // Set the default position and default heading
        position=default_position;
        turn_rate=0;
        movement_vector=new VectorF(0.0f,0.0f);

        // CONTROL HUB ORIENTATION SETTINGS
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection,usbDirection);

        // Set the IMU (Gyro) Variable
        imu=hardwareMap.get(IMU.class,"imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // Set DC Motor variables
        left_motor=hardwareMap.get(DcMotor.class,"left_motor");
        right_motor=hardwareMap.get(DcMotor.class,"right_motor");

        // MOTOR DIRECTION SETTINGS
        left_motor.setDirection(DcMotorSimple.Direction.FORWARD);
        right_motor.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    // Set default_heading to 0.0 if default_heading isn't given
    public Movement(VectorF default_position, HardwareMap hardwareMap) {this(default_position,0.0,hardwareMap);}
    public void RobotStart() {
        imu.resetYaw();
    } // Reset Yaw to 0
    public void UpdateRobot() {
        // TODO: Add position tracker
        //double turn=abs(turn_rate);
        double left_power=
                turn_rate*TURN_SCALE/2.0
                +((double)movement_vector.getData()[1])*DRIVE_SCALE/2;
        double right_power=
                turn_rate*TURN_SCALE/-2.0
                +((double)movement_vector.getData()[1])*DRIVE_SCALE/2;
        left_motor.setPower(left_power);
        right_motor.setPower(right_power);
    }
    // Get heading from the IMU (Control Hub)
    public double getHeading() {
        YawPitchRollAngles YPRAimu = imu.getRobotYawPitchRollAngles();
        return YPRAimu.getYaw(AngleUnit.RADIANS);
    }
    /**
     * @param target_turn_rate Turing speed of the robot [rad/s]
     */
    public void setTurnSpeed(double target_turn_rate) {
        turn_rate=target_turn_rate;
    }
    public void holdHeading(double target_heading) {

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
