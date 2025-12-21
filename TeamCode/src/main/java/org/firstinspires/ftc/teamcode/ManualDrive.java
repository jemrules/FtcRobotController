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
            telemetry.addData("Status", "Ready to Start");
            telemetry.update();
        }
        telemetry.addData("Build Version: ", "1.0.0.0");
        // Send to the robot movement controller Init has ended
        robotMovement.RobotStart();
        launcher.RobotStart();
        //telemetry.speak("test");
        boolean isSpunUp = false;
        boolean previousCanFire = false;
        int timeSinceLastSpin = 100;
        boolean previousButtonX = false;
        while (opModeIsActive()) {
            double right_stick_y = gamepad1.right_stick_y * MOVEMENT_STICK_SENSITIVITY;
            telemetry.addData("Launcher stick", right_stick_y);
            if(gamepad1.x && !isSpunUp && timeSinceLastSpin >= 100){
                telemetry.addData("Launcher Mode:", "Spinning Up!");
                //launcher_throttle = Clamp(launcher_throttle + 1.5 * 0.14 / 1.5, 0.0, 0.14);   

                launcher.setRPS(Configuration.LAUNCHER_SPEED);//launcher_throttle * -120.0);
                isSpunUp = true;
                timeSinceLastSpin = 0;
            }
            else if(isSpunUp && gamepad1.x && timeSinceLastSpin >= 100){
                telemetry.addData("Launcher Mode:", "Spinning Down!");
                //launcher_throttle = Clamp(launcher_throttle + -1.5 * 0.14 / 1.5, 0.0, 0.14);
                launcher.setRPS(0);//launcher_throttle * -120.0);
                isSpunUp = false;
                timeSinceLastSpin = 0;
            }
            launcher.setFeederOnOff(gamepad1.left_bumper);
            telemetry.addData("Launcher Throttle", launcher_throttle);
            // this should be a bool
            boolean canFire = (abs(-launcher.getVelocity() - 1000.0) < 30.0);
            telemetry.addData("Speed Distance (can fire)", canFire);
            telemetry.addData("Launcher Speed: ", (abs(launcher.getVelocity())));
            // if we change to true rumble
            if(previousCanFire == false && canFire) {
                gamepad1.rumble(0.3, 0.3, 250);
            }
            // if a is pressed hard brake robot
            robotMovement.setBrakeOnOff(gamepad1.a);
            // Jog Control
            robotMovement.jog_position(gamepad1.dpad_left, gamepad1.dpad_right, gamepad1.dpad_up, gamepad1.dpad_down);

            robotMovement.setTurnSpeed(gamepad1.left_stick_x); // 5 degrees/second
            robotMovement.movement_vector.put(1, -gamepad1.left_stick_y);

            // Update Robot
            launcher.UpdateRobot();
            robotMovement.UpdateRobot(telemetry);
            previousCanFire = canFire;
            timeSinceLastSpin++;
        // END OF MAINLOOP
        }
    }
}
