package org.firstinspires.ftc.teamcode.util;

// COMMENT ME!
import com.acmerobotics.dashboard.config.Config;
// IMPORTANT: COMMENT OUT ALL FTC DASHBOARD COMMANDS PRIOR TO MATCH!!!
// ALL LINES THAT NEED TO BE COMMENTED OUT TO DISABLE DASHBOARD ARE 
// PREFIXED WITH:
// COMMENT ME!


// COMMENT ME!
@Config
public class Configuration{
    // with these values launcher is really consistent
	public static double LAUNCHER_P = 50.f;
	public static double LAUNCHER_I = 4.f;
	public static double LAUNCHER_D = 20.f;
	public static double LAUNCHER_F = 1.0;
    public static double LAUNCHER_SPEED = -13.3;
    public static double FIRE_RATE_CAP = 100;
    public static double SPIN_TIME = 1000000;
    public static double TURN_MULTIPLIER = 0.75;
	// movement settings
    // SETTINGS
    // seconds to move back after firing period of auto finishes

    public static double AUTO_BACK_AMOUNT = 4.0f; 
    public static double AUTO_FORWARD_AMOUNT = 1.0f;
    public static double TURN_SCALE=1;
    public static double DRIVE_SCALE=1;
    public static double MOTOR_RPM=6000.0; // The Motors RPM
    public static double GEAR_RATIO=1.0/20.0; // The gear ratio coming out of the motor
    public static double WHEEL_DIAMETER=92.0/1000.0; // The diameter of the wheels [mm] to [m]
    public static double WHEEL_SPACING=38.0/100.0; // The distance between the left and right wheels [cm] to [m]
    public static double MAX_ACCELERATION= 1.7; // [%/s^2]
    // Servo Gate
    public static double GATE_OPEN_TIME = 0.75; // [Seconds]
    public static double GATE_OPEN_ANGLE = 90.0; // [Deg]
    public static double GATE_CLOSE_ANGLE = 0.0; // [Deg]
}