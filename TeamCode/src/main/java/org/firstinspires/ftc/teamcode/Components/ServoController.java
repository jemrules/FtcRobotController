package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.Misc.Clamp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
public class ServoController {
	private Servo servo;
	private double movementRange;
	public Launcher(HardwareMap hardwareMap, String servo_name, Servo.Direction direction, double movementRange) {
		servo = hardwareMap.get(Servo.class,servo_name);
		this.movementRange = movementRange;
	}
	// position in radians between 0 & movementRange rads (usually 3/2*pi)
	public void setPosition(double position){
		double pos = Clamp(position, 0, movementRange);
		servo.setPosition(pos);
	}
	public double getPosition(){
		return (servo.getPosition()*(movementRange));
	}
}
