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
    private DcMotorEx motor;
	public MotorController(HardwareMap hardwareMap, String name, DcMotorSimple.Direction direction){
		// dc motor ex is probably better?
		motor = (DcMotorEx)hardwareMap.get(DcMotor.class, name);
		motor.setDirection(direction);
				
	}
	public void set_mode(DcMotor.RunMode mode){
		motor.setMode(mode);
	}
	public DcMotor.RunMode getMode(){
		return motor.getMode();
	
	public DcMotor.RunMode getMode(){
		return motor.getMode();
	}
	/// returns rads/sec
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

