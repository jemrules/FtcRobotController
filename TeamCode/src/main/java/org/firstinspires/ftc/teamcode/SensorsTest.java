package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.robot.Movement;
import org.firstinspires.ftc.teamcode.robot.Sensors;

import static java.lang.Math.PI;
import static java.lang.Math.toRadians;
@TeleOp
public class SensorsTest extends LinearOpMode {
    public Sensors robotSensors;
    @Override
    public void runOpMode() {
        robotSensors = new Sensors(
                hardwareMap);
        robotSensors.startStream();
        while (opModeInInit()) {
            telemetry.addData("Status","Ready to Start");
            telemetry.update();
        }
        while (opModeIsActive()) {
            robotSensors.detectTags();
            robotSensors.telemetryOutput(telemetry);
            telemetry.update();
            sleep(20);
        }
        robotSensors.visionPortal.close();
    }
}
