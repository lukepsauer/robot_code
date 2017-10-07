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

            ArrayList<Float> vuMarkLocation = robot.getVuMarkLocation();


            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(robot.relicTemplate);

            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
                //telemetry.addData("VuMark: ", vuMark);

                /* For fun, we also exhibit the navigational pose. In the Relic Recovery game,
                 * it is perhaps unlikely that you will actually need to act on this pose information, but
                 * we illustrate it nevertheless, for completeness. */
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) robot.relicTemplate.getListener()).getPose();
                //telemetry.addData("Pose", format(pose).);

                /* We further illustrate how to decompose the pose into useful rotational and
                 * translational components */
                if (pose != null) {
                    VectorF trans = pose.getTranslation();

                    Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                    // Extract the X, Y, and Z components of the offset of the target relative to the robot
                    tX = trans.get(0);
                    tY = trans.get(1);
                    tZ = trans.get(2);

                }
            }


            // telemetry.addData("X: ", (float) tX);
            // telemetry.addData("Y: ", tY);
            // telemetry.addData("Z :", tZ)

            if (!(gamepad1.right_trigger < .1)) {
                ;

                if (robot.imu.getAngularOrientation().firstAngle > 0) {
                    robot.drive(0, Math.pow(tX/250, 1), ((Math.abs(robot.imu.getAngularOrientation().firstAngle) < 10) ? 0.1 : .25));

                } else {
                    robot.drive(0, Math.pow(tX/250, (1)), -((Math.abs(robot.imu.getAngularOrientation().firstAngle) < 10) ? .1 : .25));

                }
            } else {
                robot.stop();
            }


//            if ((gamepad1.right_trigger < .1)) {
//                robot.setMotors((float) -(gamepad1.left_stick_y), (float) (gamepad1.left_stick_x), (float) (gamepad1.right_stick_x));
//            } else {
//                robot.setMotors(-((float) tY + 60) / 500, ((float) -tX) / 125, 0);
//            }


            telemetry.addData("VELOCITY: ", robot.imu.getVelocity());
            telemetry.addData("VUMARK X: ", tX);
            telemetry.addData("HEADING: ", robot.imu.getAngularOrientation().firstAngle);
            telemetry.update();
        }
    }



    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
}