package org.firstinspires.ftc.teamcode.robot;

import org.firstinspires.ftc.teamcode.util.Configuration;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static java.lang.Math.abs;
import static java.lang.Math.max;
import static java.lang.Math.min;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.concurrent.TimeUnit;

public class Launcher {
    Movement movement;
    public double flywheel_rps=0.0;
    public boolean feeder=false;
    public boolean launched = true;
    public ElapsedTime LaunchTimer;
    public boolean gateOpen = false;
    public DcMotorEx flywheel_motor; // [RPS]
    public DcMotor feeder_motor;
    public Servo gate_servo;
    public Launcher(HardwareMap hardwareMap) {

        // map Motors
        flywheel_motor=(DcMotorEx)hardwareMap.get(DcMotor.class,"flywheel_motor");
        feeder_motor=hardwareMap.get(DcMotor.class,"feeder_motor");
        feeder_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        gate_servo=hardwareMap.get(Servo.class,"gate_servo");
        flywheel_motor.setVelocityPIDFCoefficients(Configuration.LAUNCHER_P, Configuration.LAUNCHER_I, Configuration.LAUNCHER_D, Configuration.LAUNCHER_F);
        LaunchTimer=new ElapsedTime();
    }
    public Launcher(HardwareMap hardwareMap,Movement movement_inst) {
        this(hardwareMap);
        movement = movement_inst;
    }
    public void RobotStart() {

    }
    public void UpdateRobot() {
        flywheel_motor.setVelocity(min(flywheel_rps,120.0)*60.0, AngleUnit.DEGREES);
        if(feeder && !launched){
            feeder_motor.setPower(1);
            if (LaunchTimer.seconds()<Configuration.GATE_OPEN_TIME) {
                gate_servo.setPosition(Configuration.GATE_OPEN_ANGLE/270.0);
            } else {
                gate_servo.setPosition(Configuration.GATE_CLOSE_ANGLE/270.0);
                launched=true;
            }
        }
        else {
            feeder_motor.setPower(0);
            gate_servo.setPosition(Configuration.GATE_CLOSE_ANGLE/270.0);
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
        if (state && !feeder) {
            LaunchTimer.reset();
        }
        if (launched && state) {
            feeder=true;
            launched=false;
            LaunchTimer.reset();
        } else if (feeder && !state && launched) {
            feeder=false;
        }
    }
    public void debugData(Telemetry telemetry) {
        //PIDFCoefficients pidCoefficients = flywheel_motor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        //telemetry.addData("PID Settings", String.format("P:%f I:%f D:%f F:%f", pidCoefficients.p,pidCoefficients.i,pidCoefficients.d,pidCoefficients.f));
        //telemetry.update();
        telemetry.addData("feeder",feeder);
        telemetry.addData("launched",launched);
        telemetry.addData("timer",LaunchTimer.now(TimeUnit.SECONDS));
    }
}
