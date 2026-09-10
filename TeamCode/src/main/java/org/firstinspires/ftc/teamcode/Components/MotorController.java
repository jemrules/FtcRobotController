package org.firstinspires.ftc.teamcode.robot;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import static java.lang.Math.abs;
import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.min;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;

import java.lang.reflect.Array;
import java.util.Arrays;

// this provides control of a dc motor
public class MotorController {
	public static double TURN_SCALE=1;
    public static double DRIVE_SCALE=1;
    public static double MOTOR_RPM=6000.0; // The Motors RPM
    public static double GEAR_RATIO=1.0/20.0; // The gear ratio coming out of the motor
    public static double WHEEL_DIAMETER=92.0/1000.0; // The diameter of the wheels [mm] to [m]
    public static double WHEEL_SPACING=38.0/100.0; // The distance between the left and right wheels [cm] to [m]
    public static double MAX_MOTOR_ACCELERATION=5.0/2.0;

    public VectorF position;
    public double turn_rate;
    public VectorF movement_vector;
    public DcMotorEx motor=new DcMotorEx();
	public MotorController(HardwareMap hardwareMap, DcMotorSimple.Direction direction){
		motor.setDirection(DcMotorSimple.Direction.REVERSE);
				
	}
	// returns rads/sec
	public double getVelocity(){
		return motor.getVelocity(AngleUnit.RADIANS);
	}
	// set velocity in rads/sec
	public void setVelocity(double velocity){
		motor.setVelocity(velocity, AngleUnit.RADIANS);	
	}
	public void setPIDF(float p, float i, float d, float f){
		motor.setVelocityPIDF(p, i, d, f);
	}
	// returns the new state of the motor
	public bool	toggleDisable(){
		bool enabled = motor.isMotorEnabled();
		if(enabled){
			motor.setMotorDisable();
			return 0;
		}else{
			motor.setMotorEnable();
			return 1;
		}
	}
	//returns if is disabled 
	public bool isDisabled(){
		return !motor.isMotorEnabled();
	}
	
}

