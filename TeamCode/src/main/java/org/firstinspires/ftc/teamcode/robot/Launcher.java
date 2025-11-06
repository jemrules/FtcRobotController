package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import static java.lang.Math.abs;
import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.max;
import static java.lang.Math.min;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Launcher {
    // PID Settings
    public static final double PID_P=0.0;
    public static final double PID_I=0.0;
    public static final double PID_D=0.0;

    Movement movement;
    public double flywheel_rps=0.0;

    public DcMotorEx flywheel_motor; // [RPS]
    public DcMotor feeder_motor;
    public Servo agitator_servo;
    public Launcher(HardwareMap hardwareMap) {

        // map Motors
        flywheel_motor=(DcMotorEx)hardwareMap.get(DcMotor.class,"flywheel_motor");
        feeder_motor=hardwareMap.get(DcMotor.class,"feeder_motor");

        agitator_servo=hardwareMap.get(Servo.class,"agitator_servo");
    }
    public Launcher(HardwareMap hardwareMap,Movement movement_inst) {
        this(hardwareMap);
        movement = movement_inst;
    }
    public void RobotStart() {

    }
    public void UpdateRobot() {
        flywheel_motor.setVelocity(min(flywheel_rps,120), AngleUnit.DEGREES);
    }
    public void SetRPS(double rps) {
        flywheel_rps=rps;
    }
    public double getPower() {
        return flywheel_rps / 120.0;
    }
    public void debugData(Telemetry telemetry) {
        PIDFCoefficients pidCoefficients = flywheel_motor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("PID Settings", String.format("P:%f I:%f D:%f F:%f", pidCoefficients.p,pidCoefficients.i,pidCoefficients.d,pidCoefficients.f));
        telemetry.update();
    }
}
