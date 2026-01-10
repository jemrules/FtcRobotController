package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.util.Configuration;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.robot.Launcher;
import org.firstinspires.ftc.teamcode.robot.Movement;
import org.firstinspires.ftc.teamcode.robot.Sensors;


@Autonomous
public class AutoDriveForward extends LinearOpMode {
    public Launcher launcher;
    public Movement robotMovement;
    public Sensors robotSensors;
    @Override
    public void runOpMode() {
        launcher=new Launcher(hardwareMap);
        robotMovement = new Movement(
                new VectorF(0.0f, 0.0f, 0.0f), // Set default position to 0,0,0
                hardwareMap); // Pass the ability to interact with hardware
        // robotSensors = new Sensors(
        //         hardwareMap);
        double wheel_max_rps=robotMovement.left_motor.getMotorType().getMaxRPM()/60.0;
        // Wait until the play button is pressed
        while (opModeInInit()) {
            telemetry.addData("Status", "Ready to Start");
            telemetry.update();
        }
        robotMovement.RobotStart();
        launcher.RobotStart();
        ElapsedTime drive_time = new ElapsedTime();
        drive_time.reset();
        if (opModeIsActive()) {
                robotMovement.left_motor.setPower(0.5);
                robotMovement.right_motor.setPower(0.5);
                telemetry.update();
            while(drive_time.seconds()<Configuration.AUTO_FORWARD_AMOUNT);
        }
    }
}
