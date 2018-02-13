package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.CryptoboxDetector;
import com.disnodeteam.dogecv.detectors.JewelDetector;
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


@TeleOp(name = "#Memes", group = "Test")
public class Test extends LinearOpMode {
    RobotControl robot;
    boolean red = true;
    boolean relicGrabbed=false;
    boolean armDown=false;
    boolean relicExtrude = false;
    boolean harvesterRun = false;
    int pastPosition = 0;
    int harvesterDir = 1;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new RobotControl(this);


        robot.clawServo.setPosition(.8);
        robot.jewelY.setPosition(.84);

        waitForStart();




        while (opModeIsActive()) {




            robot.setMotors( (float)(gamepad1.left_stick_y), (float) (-gamepad1.left_stick_x), (float) (gamepad1.right_stick_x));
//            robot.harvester(gamepad1.right_trigger > .1, gamepad1.right_bumper);


            if(gamepad2.dpad_up){
                relicExtrude = true;
                robot.relicM.setPower(.6);
            }
            else{
                relicExtrude = false;
                robot.relicM.setPower(0);
            }

            if(gamepad1.x){

            }
            else{
                robot.belt.setPower(0);
            }




            //jewelX Middle = .35 +- 10 for each side + left - right
            //jewelY up = .84 down = .38


            //if(gamepad1.y) robot.speedController();





                robot.relicM.setPower(Math.abs(gamepad2.left_stick_y)*.8);

            if(harvesterRun && robot.eHarvest.getCurrentPosition() == pastPosition){
                harvesterDir = -1;
            }
            else{
                harvesterDir = 1;
            }


            if(gamepad2.y){
                armDown = true;
                robot.armServo.setPosition(.2);
            }
            else{
                armDown = false;
                robot.armServo.setPosition(.6);
            }



            if(gamepad2.left_bumper){
                relicGrabbed = true;
                robot.clawServo.setPosition(.6);
            }
            else{
                relicGrabbed = false;
               robot.clawServo.setPosition(.8);
            }
           // robot.balance(gamepad1.a);



//            if(gamepad2.right_trigger>.1 && gamepad2.left_trigger >.1){
//                robot.relicM.setPower(.8);
//                robot.runtime.reset();
//                while(robot.runtime.milliseconds() < 500);
//                robot.runtime.reset();
//                robot.armServo.setPosition(.2);
//                while(robot.runtime.milliseconds() < 1000);
//                robot.runtime.reset();
//                robot.clawServo.setPosition(.6);
//                while(robot.runtime.milliseconds() < 500);
//                robot.runtime.reset();
//                robot.relicM.setPower(.2);
//                while(robot.runtime.milliseconds() < 500);
//                robot.runtime.reset();
//                robot.armServo.setPosition(.6);
//                while(robot.runtime.milliseconds() < 500);
//                robot.runtime.reset();
//                robot.relicM.setPower(0);
//            }








            if(gamepad1.right_bumper){
                robot.eHarvest.setPower(1);
                robot.wHarvest.setPower(-1);
                robot.belt.setPower(.7);

            }
            else if(gamepad1.right_trigger > .1) {

                robot.belt.setPower(-.7);
                robot.eHarvest.setPower(-1*harvesterDir);
                robot.wHarvest.setPower(1*harvesterDir);
            }
            else{
                robot.belt.setPower(0);

                robot.eHarvest.setPower(0);
                robot.wHarvest.setPower(0);
            }
            int colorTarget;
            if(gamepad1.a){

                //Selection Num red>22
                colorTarget = robot.eColor.red();
            }
            else{
                //Selection num blue>19
                colorTarget = robot.eColor.blue();
            }


            telemetry.addData("HEADING: ", robot.imu.getAngularOrientation().firstAngle);
            telemetry.addData("PITCH: ", robot.imu.getAngularOrientation().secondAngle);
            telemetry.addData("ROLL: ", robot.imu.getAngularOrientation().thirdAngle);
            telemetry.addData("COLORe", colorTarget);
            telemetry.addData("CONTROL_SPEED: ", robot.controlSpeed);


            telemetry.update();
        }

        if(!opModeIsActive()){

        }



    }


}