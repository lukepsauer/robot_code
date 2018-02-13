package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.opencv.android.CameraBridgeViewBase;

import java.util.ArrayList;


/**
 * Created by nqmet on 9/9/2017.
 */

public class RobotControl {
    public final HardwareMap hwMap;
    private LinearOpMode linearOpMode = null;
    DcMotor ne, nw, se, sw, relicM, eHarvest, wHarvest;
    BNO055IMU imu;
    VuforiaLocalizer vuforia;
    VuforiaTrackables relicTrackables;
    VuforiaTrackable relicTemplate;
    ElapsedTime runtime;
    Servo clawServo, armServo, jewelY, jewelX;
    CRServo belt;
    ColorSensor eColor, wColor;
    double controlSpeed = .1;


//    RobotControl(OpMode opMode){
//        hwMap = opMode.hardwareMap;
//        defineVariables(hwMap);
//    }

    RobotControl(LinearOpMode opMode) {



        linearOpMode = opMode;
        hwMap = linearOpMode.hardwareMap;
        runtime = new ElapsedTime();

        this.ne = this.hwMap.dcMotor.get("ne");
        this.nw = this.hwMap.dcMotor.get("nw");
        this.se = this.hwMap.dcMotor.get("se");
        this.sw = this.hwMap.dcMotor.get("sw");
        this.relicM = this.hwMap.dcMotor.get("relicExtrude");
        this.eHarvest = this.hwMap.dcMotor.get("eHarvester");
        this.wHarvest = this.hwMap.dcMotor.get("wHarvester");
        this.clawServo = this.hwMap.servo.get("clawServo");
        this.armServo = this.hwMap.servo.get("armServo");
        this.eColor =this.hwMap.colorSensor.get("eColor");
        this.wColor = this.hwMap.colorSensor.get("wColor");
        this.jewelX = this.hwMap.servo.get("jewelX");
        this.jewelY = this.hwMap.servo.get("jewelY");
        this.belt = this.hwMap.crservo.get("belt");



        this.nw.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.se.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ne.setDirection(DcMotorSimple.Direction.REVERSE);
        nw.setDirection(DcMotorSimple.Direction.REVERSE);
        se.setDirection(DcMotorSimple.Direction.REVERSE);
        sw.setDirection(DcMotorSimple.Direction.REVERSE);
        ne.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        se.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sw.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        nw.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        relicM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        this.imu = this.hwMap.get(BNO055IMU.class, "imu");

//        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
//        VuforiaLocalizer.Parameters vuParameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
//
//
//        vuParameters.vuforiaLicenseKey = "AWiPmUf/////AAAAGf+WLNJhFkkwrVBlxb9zE91rUnpsG6NTvqENMyoh4egDWwWB/gcZ2651s6F986PKNAS3SxorAtRe6PprAWSuslsabqF05Cv1v0EE6XHZnLWuDm68eajzFFS3Mv0qULYqjqvDSIUSZ3WSNK5f1PmVOSrUb8lGH910r8hxmtQwV+1QxucaiMYtI5nEJFnFkJ2TcHbDbcP7qKF2rcPQb44dZOZCqBkYsy0WV78PYQums4WTiHt8pBmRjHJDGVo7Biyqk6DNdzWx49QhKyHpV5xlTfoz7ls0DzatmJxdqJtJnsa96EdeDRuomkiVqTAH+68B6tlKeE7J+SNVIk//xdRtXHFQt5jATJ9rNtYW87of1/tH";
//
//
//        vuParameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
//        this.vuforia = ClassFactory.createVuforiaLocalizer(vuParameters);
//
//
//        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
//        relicTemplate = relicTrackables.get(0);
//        relicTemplate.setName("relicVuMarkTemplate");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu.initialize(parameters);

        this.jewelY.setPosition(.84);





    }



    // RELIC ARM SERVO NUMS

    //ARM: .6 = up down=.2
    //CLAW .8 = close open = .6




    public String getVuMarkState() {
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

        if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
            return vuMark.toString();
        } else {
            //  telemetry.addData("VuMark", "not visible");
            return "NONE";
        }
    }

    public ArrayList<Float> getVuMarkLocation(){
        ArrayList<Float> location = new ArrayList<Float>();


        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

        if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
            //telemetry.addData("VuMark: ", vuMark);

                /* For fun, we also exhibit the navigational pose. In the Relic Recovery game,
                 * it is perhaps unlikely that you will actually need to act on this pose information, but
                 * we illustrate it nevertheless, for completeness. */
            OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose();
            //telemetry.addData("Pose", format(pose).);

                /* We further illustrate how to decompose the pose into useful rotational and
                 * translational components */
            if (pose != null) {
                VectorF trans = pose.getTranslation();

                Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                // Extract the X, Y, and Z components of the offset of the target relative to the robot
                float tX = trans.get(0);
                float tY = trans.get(1);
                float tZ = trans.get(2);

                float rX = rot.firstAngle;
                double rY = rot.secondAngle;
                double rZ = rot.thirdAngle;


                location.add(tX);
                location.add(tY);
                location.add(tZ);
                location.add(rX);




                // telemetry.addData("X: ", (float) tX);
                // telemetry.addData("Y: ", tY);
                // telemetry.addData("Z :", tZ);


                    return location;


                // Extract the rotational components of the target relative to the robot
//
            }
            else{
                return null;
            }
        } else {
            //  telemetry.addData("VuMark", "not visible");
            return null;
        }

    }
    public void harvester(boolean in, boolean out){
        if (in) {
            this.eHarvest.setPower(.65);
            this.wHarvest.setPower(-.65);
        } else if (out) {
            this.eHarvest.setPower(-.65);
            this.wHarvest.setPower(.65);
        } else {
            this.eHarvest.setPower(0);
            this.wHarvest.setPower(0);
        }
    }
    public boolean align(){
        int limit = 3;
        double power = .8;
        while(this.imu.getAngularOrientation().firstAngle <limit*-1 &&this.imu.getAngularOrientation().firstAngle > limit){
            if(this.imu.getAngularOrientation().firstAngle<limit*-1){
                this.drive(0,0,power);
            }
            else if(this.imu.getAngularOrientation().firstAngle > limit){
                this.drive(0,0,.8*-1);
            }

        }
        this.stop();
        return true;
    }

    public void balance(boolean balance) {
        float pitch = imu.getAngularOrientation().secondAngle;
        float roll = imu.getAngularOrientation().thirdAngle;
        if (pitch > 2){
            drive(0, .2, 0);
        }
        if (pitch < -2) {
            drive(180, .2, 0 );
        }
        if (roll > 2) {
            drive(90, .2, 0);
        }
        if (roll < -2) {
            drive(270, .2, 0);
        }
    }
    public void relicExtrude(float relicMotor){
        this.relicM.setPower(relicMotor *.55);
    }

//    public void flippyDoo(boolean up) {
//        if (up){
//            this.flipServo.setPosition(.9);
//        }
//        else this.flipServo.setPosition(0);
//    }
    public void speedController(){
           controlSpeed += .3;
           if (controlSpeed == 1.3) {
               controlSpeed = .1;
           }

    }

    public void drive(double angle, double power, double rot) {
        setMotors((float) (power * Math.sin(Math.toRadians(angle))), (float) (power * -Math.cos(Math.toRadians(angle))), (float) rot);
    }

    public void stop(){
        setMotors(0,0,0);
    }

    public void setMotors(float x, float y, float rot) {
        double drive = (double) y, strafe = (double) x, spin = (double) rot;
        double nePower, nwPower, sePower, swPower;

        nwPower = Range.clip(drive + strafe + spin, -1, 1);
        swPower = Range.clip(drive - strafe + spin, -1, 1);
        nePower = Range.clip(drive - strafe - spin, -1, 1);
        sePower = Range.clip(drive + strafe - spin, -1, 1);
        nePower = -(nePower);
        sePower = -(sePower);
//        nwPower = -(nwPower);
//        swPower = -(swPower);

        //from here on is just setting motor values
        ne.setPower(nePower);
        se.setPower(sePower);
        sw.setPower(swPower);
        nw.setPower(nwPower);

    }
    private void defineVariables(HardwareMap _hwMap){
        HardwareMap hwMap = _hwMap;
    }


    public void timeout(int millis){
        double startTime = runtime.milliseconds();
        while(runtime.milliseconds() < startTime + millis);
        return;
    }
}




