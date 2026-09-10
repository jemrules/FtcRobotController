package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.util.Misc.Clamp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.robot.Launcher;
import org.firstinspires.ftc.teamcode.robot.Movement;
import org.firstinspires.ftc.teamcode.robot.Sensors;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

@TeleOp
public class ManualDrive extends LinearOpMode {
    static double LAUNCHER_STICK_SENSITIVITY =8.0;
    static double MOVEMENT_STICK_SENSITIVITY = -1.f;
    public Launcher launcher;
    public Movement robotMovement;
    public Sensors robotSensors;
    public double launcher_throttle=0.0;
    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer
    @Override
    public void runOpMode() {
//        launcher=new Launcher(hardwareMap);
        robotMovement = new Movement(
                new VectorF(0.0f, 0.0f, 0.0f), // Set default position to 0,0,0
                hardwareMap); // Pass the ability to interact with hardware
//        robotSensors = new Sensors(
//                hardwareMap);
        // Wait until the play button is pressed
        while (opModeInInit()) {
            telemetry.addData("Status", "Ready to Start");
            telemetry.update();
        }

        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        // Send to the robot movement controller Init has ended
        robotMovement.RobotStart();
//        launcher.RobotStart();
        odo.setOffsets(-84.0, -168.0, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();

        while (opModeIsActive()) {
            double right_stick_y=gamepad1.right_stick_y*MOVEMENT_STICK_SENSITIVITY;
            telemetry.addData("stick",right_stick_y);
//            launcher_throttle = Clamp(right_stick_y*0.15,0.0,0.15);

//            telemetry.addData("speed",launcher_throttle);

//            launcher.setRPS(launcher_throttle*-120.0);
//            launcher.setFeederOnOff(gamepad1.left_bumper);
            telemetry.addData("Status", "Initialized");
            telemetry.addData("X offset", odo.getXOffset(DistanceUnit.MM));
            telemetry.addData("Y offset", odo.getYOffset(DistanceUnit.MM));
            telemetry.addData("Device Version Number:", odo.getDeviceVersion());
            telemetry.addData("Heading Scalar", odo.getYawScalar());
            telemetry.update();
            robotMovement.setTurnSpeed(gamepad1.right_stick_x); // 5 degrees/second
            robotMovement.movement_vector.put(0, gamepad1.left_stick_x);
            robotMovement.movement_vector.put(1, gamepad1.left_stick_y);

            // Update Robot
//            launcher.UpdateRobot();
            robotMovement.UpdateRobot(telemetry);
        }
    }
}
