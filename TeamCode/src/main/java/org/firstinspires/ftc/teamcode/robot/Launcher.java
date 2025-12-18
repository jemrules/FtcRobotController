package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
    public static final double PID_P=3.0;
    public static final double PID_I=2;
    public static final double PID_D=1.5;

    Movement movement;
    public double flywheel_rps=0.0;
    public boolean feeder=false;

    public DcMotorEx flywheel_motor; // [RPS]
    public DcMotor feeder_motor;
    public Servo agitator_servo;
    public Launcher(HardwareMap hardwareMap) {

        // map Motors
        flywheel_motor=(DcMotorEx)hardwareMap.get(DcMotor.class,"flywheel_motor");

        feeder_motor=hardwareMap.get(DcMotor.class,"feeder_motor");
        feeder_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        agitator_servo=hardwareMap.get(Servo.class,"agitator_servo");
    }
    public Launcher(HardwareMap hardwareMap,Movement movement_inst) {
        this(hardwareMap);
        movement = movement_inst;
    }
    public void RobotStart() {

    }
    public void UpdateRobot() {
        flywheel_motor.setVelocity(min(flywheel_rps,120.0)*60.0, AngleUnit.DEGREES);
        if (feeder) {
            feeder_motor.setPower(1);
        } else {
            feeder_motor.setPower(0);
        }
    }
    public void setRPS(double rps) {
        flywheel_rps=rps;
    }
    public double getPower() {
        return flywheel_rps / 120.0;
    }
    // returns launcher speed in deg/sec
    public double getVelocity(){return flywheel_motor.getVelocity(AngleUnit.DEGREES);}
    public void setFeederOnOff(boolean state) {
        feeder=state;
    }
    public void debugData(Telemetry telemetry) {
        PIDFCoefficients pidCoefficients = flywheel_motor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("PID Settings", String.format("P:%f I:%f D:%f F:%f", pidCoefficients.p,pidCoefficients.i,pidCoefficients.d,pidCoefficients.f));
        telemetry.update();
    }
}
