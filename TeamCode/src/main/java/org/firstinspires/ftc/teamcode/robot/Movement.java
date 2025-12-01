package org.firstinspires.ftc.teamcode.robot;

import static org.firstinspires.ftc.teamcode.util.Misc.Clamp;
import static org.firstinspires.ftc.teamcode.util.TrigMath.subtractAngles;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import static java.lang.Math.abs;
import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.min;
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

    public double MOTOR_SMOOTHING=10.0;

    public VectorF position;
    public double turn_rate;
    public VectorF movement_vector;
    public IMU imu;
    public DcMotorEx[] motors;
    public double[] motors_power={
            0.0, 0.0,
            0.0, 0.0
    };
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
        //movementPID = PIDController(0.5, 0, 0, );
    }
    // Set default_heading to 0.0 if default_heading isn't given
    public Movement(VectorF default_position, HardwareMap hardwareMap) {this(default_position,0.0,hardwareMap);}
    public void RobotStart() {
        imu.resetYaw();
    } // Reset Yaw to 0
    public void UpdateRobot(Telemetry telemetry) {
        // TODO: Add position tracker

        telemetry.update();
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
        double INTERVAL=0.001;
        double current_heading=getHeading();
        double turn_diff=subtractAngles(current_heading,target_heading);
        setTurnSpeed(turn_diff*INTERVAL);
    }
    /**
     * @param target_position Target global 2D position [m]
     */
    public void lookToward(VectorF target_position) {
        VectorF targetVector=target_position.subtracted(position);
        double angle2target=atan2(targetVector.get(0),targetVector.get(1));
        holdHeading(angle2target);
    }
    // Drive in target_vector direction (Target Vector is relative to the robots location)
    public void driveTo(VectorF target_vector,double speed,boolean turn_towards) {
        double heading=getHeading();
        double angle2target = atan2(target_vector.get(0), target_vector.get(1));
        if (turn_towards) {
            holdHeading(angle2target);
        }
        movement_vector=new VectorF((float)(cos(angle2target-heading)*speed),(float)(sin(angle2target-heading)*speed));
    }
    public void driveTo(double orientation,double distance,double speed,boolean turn_towards) {
        driveTo(new VectorF(
                (float)(cos(orientation)*distance),
                (float)(sin(orientation)*distance)),
                speed,
                turn_towards);
    }
}
