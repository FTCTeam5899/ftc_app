package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

abstract public class AutoSupplies extends LinearOpMode{

    //  Establish hardware
    protected DcMotor motorFwdLeft;
    protected DcMotor motorFwdRight;
    protected DcMotor motorBackLeft;
    protected DcMotor motorBackRight;
    protected DcMotor lift;
    protected Servo mServo;
    protected RevBlinkinLedDriver lights;
    protected BNO055IMU imu;
    protected Orientation lastAngles = new Orientation();
    //  Establish detector
    protected GoldAlignDetector goldDetector;
    protected SamplingOrderDetector orderDetector;

    //  Declare OpMode Members
    protected ElapsedTime runtime = new ElapsedTime();
    abstract public void runOpMode() throws InterruptedException;

    //  Protected variables
    protected double l = 0.4;
    protected double r = -0.4;
    protected double globalAngle;

    private static final double COUNTS_PER_MOTOR_REV = 1680;    // Neverest 60 motor encoder
    private static final double DRIVE_GEAR_REDUCTION1 = 1.0;     // This is < 1.0 if geared UP
    private static final double COUNTS_PER_DEGREE1 = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION1) / 360;
    // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
    // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
    // and named "imu".


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

    //sets motors power
    public void setPower(double leftPow, double rightPow) {
        l = leftPow;
        r = rightPow;
        motorFwdLeft.setPower(l);
        motorFwdRight.setPower(-r);
        motorBackLeft.setPower(-l);
        motorBackRight.setPower(r);
    }

    //turns on camera and starts gold detector
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

    //turns on camera and starts order detector
    public void enableOrderDetector(){
        orderDetector = new SamplingOrderDetector();
        orderDetector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        orderDetector.useDefaults();

        orderDetector.downscale = 0; // How much to downscale the input frames

        // Optional Tuning
        orderDetector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        orderDetector.maxAreaScorer.weight = 0.005;

        orderDetector.ratioScorer.weight = 5;
        orderDetector.ratioScorer.perfectRatio = 0;

        orderDetector.enable();
    }

    //  Pause for a specified time (time: mili secs)
    public void pause(long millis){
        runtime.reset();
        while (opModeIsActive() && runtime.milliseconds() <= millis){

        }
    }
    //Turns Robot a certain number of degrees
    public void turn(int degrees, double power){
        int left = 1;
        int right = 1;
        resetAngle();
        telemetry.addData("Angle",getAngle());
        telemetry.update();
        if(degrees >= 0){
            left *= -1;
        }
        else if(degrees < 0){
            right *= -1;
        }

        motorBackLeft.setPower(power * left);
        motorFwdLeft.setPower(-power * left);
        motorBackRight.setPower(-power * right);
        motorFwdRight.setPower(power* right);

        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() >= degrees) {telemetry.addData("Angle2",getAngle());telemetry.update();}
        }
        else    // left turn.
            while (opModeIsActive() && getAngle() < degrees) {telemetry.addData("Angle2",getAngle());telemetry.update();}

        // turn the motors off.
        motorBackLeft.setPower(0);
        motorFwdLeft.setPower(0);
        motorBackRight.setPower(0);
        motorFwdRight.setPower(0);
    }

    // turns the robot to a set angle
    public void turnTo(int degrees, double power){
        int left = 1;
        int right = 1;
        telemetry.addData("Angle3",getAngle());
        telemetry.update();
        if(getAngle() >= degrees){
            left *= -1;
        }
        else if(getAngle() < degrees){
            right *= -1;
        }

        motorBackLeft.setPower(power * left);
        motorFwdLeft.setPower(-power * left);
        motorBackRight.setPower(-power * right);
        motorFwdRight.setPower(power* right);

        if (getAngle() > degrees)
        {
            // On left turn we have to get off zero first.
            while (opModeIsActive() && getAngle() >= degrees) {
                telemetry.addData("Angle4",getAngle());
                telemetry.update();
            }
        }
        else {    // right turn.
            while (opModeIsActive() && getAngle() <= degrees) {
                telemetry.addData("Angle4", getAngle());
                telemetry.update();
            }
        }
        // turn the motors off.
        motorBackLeft.setPower(0);
        motorFwdLeft.setPower(0);
        motorBackRight.setPower(0);
        motorFwdRight.setPower(0);
    }
    //Resets gyro sensor to 0
    public void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    public double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    //drives the robot in a straight line using the gyro sensor
    public void moveStraight(long millis, double power){
        runtime.reset();
        if(power >= 0.9 || power <= -0.9){
            power *= 0.9;
        }
        double left = power;
        double right = power;
        while(!isStopRequested() && runtime.milliseconds() <= millis) {
            motorBackLeft.setPower(-left);
            motorFwdLeft.setPower(left);
            motorBackRight.setPower(right);
            motorFwdRight.setPower(-right);
            if(getAngle() >= 1){
                if(power>=0) {
                    right *= 0.99;
                }
                else{
                    left *= 0.99;
                }
            }
            else if(getAngle() <= -1){
                if(power>=0) {
                    left *= 0.99;
                }
                else{
                    right *= 0.99;
                }
            }
            else{
                left = power;
                right = power;
            }
            telemetry.addData("Angle", getAngle());
            telemetry.addData("LPower",left);
            telemetry.addData("RPower",right);
            telemetry.update();
        }
        motorBackLeft.setPower(0);
        motorFwdLeft.setPower(0);
        motorBackRight.setPower(0);
        motorFwdRight.setPower(0);

    }

    //  Init all hardware
    public void initForAutonomous()
    {

        BNO055IMU.Parameters gyroParameters = new BNO055IMU.Parameters();

        gyroParameters.mode                = BNO055IMU.SensorMode.IMU;
        gyroParameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        gyroParameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        gyroParameters.loggingEnabled      = false;

        motorFwdRight = hardwareMap.get(DcMotor.class, "motorFwdRight");
        motorBackLeft = hardwareMap.get(DcMotor.class, "motorBackLeft");
        motorFwdLeft = hardwareMap.get(DcMotor.class, "motorFwdLeft");
        motorBackRight = hardwareMap.get(DcMotor.class, "motorBackRight");
        lift = hardwareMap.get(DcMotor.class, "lift");
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFwdRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFwdLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        mServo = hardwareMap.get(Servo.class, "mServo");
        lights = hardwareMap.get(RevBlinkinLedDriver.class, "lights");
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(gyroParameters);

        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);

        telemetry.clear();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

}
