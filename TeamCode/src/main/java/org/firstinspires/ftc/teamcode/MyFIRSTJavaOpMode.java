package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.robot.Movement;

/* Config List:
     device -> name
     --------------
     IMU -> imu
     Left HD Motor -> left_motor
     Right HD Motor -> right_motor

*/
@TeleOp
public class MyFIRSTJavaOpMode extends LinearOpMode {
    public Movement robotMovement;
    @Override
    public void runOpMode() {
        robotMovement =new Movement(
                new VectorF(0.0f,0.0f,0.0f), // Set default position to 0,0,0
                hardwareMap); // Pass the ability to interact with hardware

    }
}
