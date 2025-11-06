package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.robot.Launcher;
import org.firstinspires.ftc.teamcode.robot.Movement;
import org.firstinspires.ftc.teamcode.robot.Sensors;

@TeleOp
public class ManualDrive extends LinearOpMode {
    public Launcher launcher;
    public Movement robotMovement;
    public Sensors robotSensors;
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
            launcher.setRPS(gamepad1.right_stick_y*120.0);
            launcher.setFeederOnOff(gamepad1.left_bumper);

            robotMovement.setTurnSpeed(gamepad1.left_stick_x); // 5 degrees/second
            robotMovement.movement_vector.put(1, gamepad1.left_stick_y);

            // Update Robot
            launcher.UpdateRobot();
            robotMovement.UpdateRobot();
        }
    }
}
