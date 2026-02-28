package org.firstinspires.ftc.teamcode;
// FTC SDK telemetry
import org.firstinspires.ftc.robotcore.external.Telemetry;

// FTC Dashboard

// COMMENT ME!
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
// DONT COMMENT BELOW ME!  
import static org.firstinspires.ftc.teamcode.util.Misc.Clamp;
import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.Configuration;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.robot.Launcher;
import org.firstinspires.ftc.teamcode.robot.Movement;
import org.firstinspires.ftc.teamcode.robot.Sensors;

@TeleOp
public class ManualDrive extends LinearOpMode {
    //static double LAUNCHER_STICK_SENSITIVITY =8.0;
    // must be negative. 
    public Launcher launcher;
    public Movement robotMovement;
    public Sensors robotSensors;
    public double launcher_throttle=0.0;
    public boolean rumble=false;
    @Override
    public void runOpMode() {
        launcher=new Launcher(hardwareMap);
        robotMovement = new Movement(
                new VectorF(0.0f, 0.0f, 0.0f), // Set default position to 0,0,0
                hardwareMap); // Pass the ability to interact with hardware
        robotSensors = new Sensors(
                hardwareMap);
        // COMMENT ME!
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry telemetry = new MultipleTelemetry(
                    this.telemetry,
                    dashboard.getTelemetry()
        );
        // DONT COMMENT BELOW ME!

        // Wait until the play button is pressed
        while (opModeInInit()) {
        //    telemetry.addData("Status", "Ready to Start");
         //   telemetry.update();
        ;
		}
        //telemetry.addData("Build Version: ", "1.0.0.0");
        // Send to the robot movement controller Init has ended
        robotMovement.RobotStart();
        launcher.RobotStart();
        //telemetry.speak("test");
        boolean isSpunUp = false;
        boolean previousCanFire = false;
        int timeSinceLastSpin = 100;
        boolean previousButtonX = false;
// start by spinning up
        if(opModeIsActive()){
            telemetry.addData("Launcher Mode:", "Spinning Up!");
                //launcher_throttle = Clamp(launcher_throttle + 1.5 * 0.14 / 1.5, 0.0, 0.14);   

            launcher.setRPS(Configuration.LAUNCHER_SPEED);//launcher_throttle * -120.0);
            isSpunUp = true;
            timeSinceLastSpin = 0;
        }
        while (opModeIsActive()) {
			// this is black magic dont touch this!!!!
			// I dont know why but it works really reliably so we should keep it
            if(gamepad1.x && !isSpunUp && timeSinceLastSpin >= 100){
                telemetry.addData("Launcher Mode:", "Spinning Up!"); 
                launcher.setRPS(Configuration.LAUNCHER_SPEED);
                isSpunUp = true;
                timeSinceLastSpin = 0;
            }
            else if(isSpunUp && gamepad1.x && timeSinceLastSpin >= 100){
                telemetry.addData("Launcher Mode:", "Spinning Down!");

                launcher.setRPS(0);//launcher_throttle * -120.0);
                isSpunUp = false;
                timeSinceLastSpin = 0;
            }
            // update the state of the launcher based on the gamepad bumper
            launcher.setFeederOnOff(gamepad1.left_bumper);
            // update telemetry
            telemetry.addData("Launcher Throttle", launcher_throttle);
            // this should be a bool
            // this is obselete because it's based on the old system of launcher speed
            boolean canFire = (abs(-launcher.getVelocity() - 1000.0) < 30.0);
            // telemetry
            telemetry.addData("Speed Distance (can fire)", canFire);
            // telemetry
            telemetry.addData("Launcher Speed: ", (abs(launcher.getVelocity())));
            // if we change to true rumble
            // this rumble code never runs?
            // commenting out so no way can distract drivers in match
            //    if(previousCanFire == false && canFire) {
            //        gamepad1.rumble(0.3, 0.3, 250);
            //      }   
            // if a is pressed hard brake robot
            // uses ZeroPowerBehavior set to break
            robotMovement.setBrakeOnOff(gamepad1.a);
            // Jog Control
            robotMovement.jog_position(gamepad1.dpad_left, gamepad1.dpad_right, gamepad1.dpad_up, gamepad1.dpad_down);
            // turns robot based on stick input
            robotMovement.setTurnSpeed(gamepad1.left_stick_x*Configuration.TURN_MULTIPLIER); // 5 degrees/second
            // movement vector used to have eventual compatibility w mechanum
            robotMovement.movement_vector.put(1, -gamepad1.left_stick_y);

            // Update Robot
            launcher.UpdateRobot();
            // adds some extra data to telemetry
            launcher.debugData(telemetry);
            // Update Movement
            robotMovement.UpdateRobot(telemetry);
            // This code is deprecated
            previousCanFire = canFire;
            // used for spin up button
            timeSinceLastSpin++;
        // END OF MAINLOOP
        }
    }
}
