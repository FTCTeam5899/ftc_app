package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

abstract public class AutoSupplies extends LinearOpMode{

    //  Establish hardware
    protected DcMotor motorFwdLeft;
    protected DcMotor motorFwdRight;
    protected DcMotor motorBackLeft;
    protected DcMotor motorBackRight;

    //  Establish detector
    protected GoldAlignDetector goldDetector;

    //  Declare OpMode Members
    protected ElapsedTime runtime = new ElapsedTime();
    abstract public void runOpMode() throws InterruptedException;

    //  Protected variables
    protected double l = 0.4;
    protected double r = -0.4;



    //Callable methods

    //  Move for a specified amount of time (time: milli secs, left power: {1,-1}, right power: {-1,1})
    public void move(long millis, double leftPow, double rightPow) {
        runtime.reset();
        while (opModeIsActive() && runtime.milliseconds() <= millis) {
            l = leftPow;
            r = rightPow;
            motorFwdLeft.setPower(l);
            motorFwdRight.setPower(-r);
            motorBackLeft.setPower(-l);
            motorBackRight.setPower(r);
        }
        motorFwdLeft.setPower(0);
        motorFwdRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
    }

    //  Enable Detector
    public void enableGoldDetector(){
        goldDetector = new GoldAlignDetector();
        goldDetector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        goldDetector.useDefaults();
        // Optional Tuning
        goldDetector.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        goldDetector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
        goldDetector.downscale = 0; // How much to downscale the input frames
        goldDetector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //goldDetector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        goldDetector.maxAreaScorer.weight = 0.005;
        goldDetector.ratioScorer.weight = 5;
        goldDetector.ratioScorer.perfectRatio = 0;
        goldDetector.enable();
    }

    //  Move for a specified distance (distance: in, power: -1-1)
    //public void moveDistance(double in, double power){
    //    runtime.reset();
    //    long millis = 0;
    //    millis = (long)(1000.0 * in / 27.5);
    //    while (opModeIsActive() && runtime.milliseconds() <= millis) {
    //        motorFwdLeft.setPower(power);
    //        motorFwdRight.setPower(-power);
    //        motorBackLeft.setPower(-power);
    //        motorBackRight.setPower(power);
    //    }
    //    motorFwdLeft.setPower(0);
    //    motorFwdRight.setPower(0);
    //    motorBackLeft.setPower(0);
    //    motorBackRight.setPower(0);
    //}

    //  Pause for a specified time (time: mili secs)
    public void pause(long millis){
        runtime.reset();
        while (opModeIsActive() && runtime.milliseconds() <= millis){

        }
    }

    //  Init all hardware
    public void initForAutonomous()
    {
        motorFwdRight = hardwareMap.get(DcMotor.class, "motorFwdRight");
        motorBackLeft = hardwareMap.get(DcMotor.class, "motorBackLeft");
        motorFwdLeft = hardwareMap.get(DcMotor.class, "motorFwdLeft");
        motorBackRight = hardwareMap.get(DcMotor.class, "motorBackRight");
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFwdRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFwdLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

}
