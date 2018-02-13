package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.JewelDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by nqmet on 1/19/2018.
 */
@Autonomous(name = "Crap", group = "Test")
public class Auto extends LinearOpMode {
    JewelDetector jewelDetector = null;
    RobotControl robot = new RobotControl(this);
    public void runOpMode() throws InterruptedException {
        jewelDetector = new JewelDetector();
        jewelDetector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());

        //Jewel Detector Settings
        jewelDetector.areaWeight = 0.02;
        jewelDetector.detectionMode = JewelDetector.JewelDetectionMode.MAX_AREA; // PERFECT_AREA
        //jewelDetector.perfectArea = 6500; <- Needed for PERFECT_AREA
        jewelDetector.debugContours = true;
        jewelDetector.maxDiffrence = 15;
        jewelDetector.ratioWeight = 15;
        jewelDetector.minArea = 700;

        jewelDetector.enable();
        waitForStart();

        while(opModeIsActive()){

            String[] order = jewelDetector.getLastOrder().toString().split("_");
            robot.runtime.reset();
//            while(robot.runtime.milliseconds() < 1000){
//                robot.jewelServo.setPower(.8);
//            }
//            robot.jewelServo.setPower(.5);

            telemetry.addData("Order", order);

        }
    }
}
