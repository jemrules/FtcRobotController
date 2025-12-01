package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.util.Misc.Clamp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.robot.Launcher;
import org.firstinspires.ftc.teamcode.robot.Movement;
import org.firstinspires.ftc.teamcode.robot.Sensors;

@TeleOp
public class ManualDrive extends LinearOpMode {
    static double LAUNCHER_STICK_SENSITIVITY =8.0;
    static double MOVEMENT_STICK_SENSITIVITY = -1.f;
    public Launcher launcher;
    public Movement robotMovement;
    public Sensors robotSensors;
    public double launcher_throttle=0.0;
    @Override
    public void runOpMode() {
        launcher=new Launcher(hardwareMap);
        robotMovement = new Movement(
                new VectorF(0.0f, 0.0f, 0.0f), // Set default position to 0,0,0
                hardwareMap); // Pass the ability to interact with hardware
        robotSensors = new Sensors(
                hardwareMap);
        // Wait until the play button is pressed
        while (opModeInInit()) {
            telemetry.addData("Status", "Ready to Start");
            telemetry.update();
        }
        // Send to the robot movement controller Init has ended
        robotMovement.RobotStart();
        launcher.RobotStart();

        while (opModeIsActive()) {
            double right_stick_y=gamepad1.right_stick_y*MOVEMENT_STICK_SENSITIVITY;
            telemetry.addData("stick",right_stick_y);
            launcher_throttle = Clamp(right_stick_y*0.15,0.0,0.15);

            double currentRPS = launcher.getRPS();
            // if the motor is off dont show speed
            // we have 0.1 instead of 0 because of errors in rpm sensor
            if(currentRPS <= 0.1) {
                telemetry.addData("speed", launcher_throttle);
                launcher.setRPS(launcher_throttle * -120.0);
                launcher.setFeederOnOff(gamepad1.left_bumper);

                telemetry.addData("Launcher charge: ", launcher.getPercentage());
                if(launcher.getPercentage() > 95) {
                    // rumble the controller for ___ ms when the launcher is at speed
                    gamepad1.rumble(500);
                }
            }

            robotMovement.setTurnSpeed(gamepad1.left_stick_x); // 5 degrees/second
            robotMovement.movement_vector.put(1, gamepad1.left_stick_y);

            // Update Robot
            launcher.UpdateRobot();
            robotMovement.UpdateRobot(telemetry);
        }
    }
}
