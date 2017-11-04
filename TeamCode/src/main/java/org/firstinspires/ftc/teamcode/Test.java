package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;

import java.util.ArrayList;


/**
 * Created by nqmet on 9/23/2017.
 */


@TeleOp(name = "Test_Bot", group = "Test")
public class Test extends LinearOpMode {
    RobotControl robot;
    boolean TrackingLock1 = false, TrackingLock2 = false;
    float X;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotControl(this);
        double tX = 0, tY = 0, tZ = 0;
        waitForStart();
        robot.relicTrackables.activate();
        while (opModeIsActive()) {
            
            robot.setMotors((float) (gamepad1.left_stick_x), (float) (gamepad1.left_stick_y), (float) (gamepad1.right_stick_x));



            telemetry.addData("HEADING: ", robot.imu.getAngularOrientation().firstAngle);
            telemetry.addData("PITCH: ", robot.imu.getAngularOrientation().secondAngle);
            telemetry.addData("ROLL: ", robot.imu.getAngularOrientation().thirdAngle);

            telemetry.update();
        }
    }
}