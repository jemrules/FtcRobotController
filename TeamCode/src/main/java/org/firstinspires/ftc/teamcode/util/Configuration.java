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
    public static double LAUNCHER_SPEED = -13;
	// movement settings
    // SETTINGS
    // seconds to move back after firing period of auto finishes 
    public static double MOVE_BACK_AMOUNT = 3.0; 
    public static double TURN_SCALE=1;
    public static double DRIVE_SCALE=1;
    public static double MOTOR_RPM=6000.0; // The Motors RPM
    public static double GEAR_RATIO=1.0/20.0; // The gear ratio coming out of the motor
    public static double WHEEL_DIAMETER=92.0/1000.0; // The diameter of the wheels [mm] to [m]
    public static double WHEEL_SPACING=38.0/100.0; // The distance between the left and right wheels [cm] to [m]

    public static double MAX_ACCELERATION= 1.0; // [%/s^2]
}