package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
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
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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
    protected Rev2mDistanceSensor distanceSensorL;
    protected Rev2mDistanceSensor distanceSensorR;
    protected RevBlinkinLedDriver lights;
    protected BNO055IMU imu;
    protected Orientation lastAngles = new Orientation();
    protected Orientation lastPitches = new Orientation();
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
    protected double globalPitch;

    //Encoder Specs
    //We keep these motors separate in case a change is made to motors or gear boxes

    //  Neverest 40 motor spec:  quadrature encoder, 280 pulses per revolution, count = 280 *4
    private static final double COUNTS_PER_MOTOR_REV = 1120;    // Neverest 40 motor encoder                    Left(shoulder) motor
    private static final double DRIVE_GEAR_REDUCTION1 = 13.5;     // This is < 1.0 if geared UP
    private static final double COUNTS_PER_DEGREE1 = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION1) / 360;

    //  Neverest 40 motor right spec:  quadrature encoder, 280 pulses per revolution, count = 280 *4
    private static final double COUNTS_PER_MOTOR_REV2 = 1120;    // Neverest 40 motor encoder                   Right(elbow) motor
    private static final double DRIVE_GEAR_REDUCTION2 = 13.5;     // This is < 1.0 if geared UP
    private static final double COUNTS_PER_DEGREE2 = (COUNTS_PER_MOTOR_REV2 * DRIVE_GEAR_REDUCTION2) / 360;

    //  Neverest 60 motor left spec:  quadrature encoder, 420 pulses per revolution, count = 420 *4             We do not currently use 60's but have in the past
    //private static final double COUNTS_PER_MOTOR_REV = 1680;    // Neverest 60 motor encoder                  This is just a reserve in case changes are made
    //private static final double DRIVE_GEAR_REDUCTION1 = 27.0;     // This is < 1.0 if geared UP
    //private static final double COUNTS_PER_DEGREE1 = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION1) / 360;


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

    //sets motors power to a certain value
    public void setPower(double leftPow, double rightPow) {
        l = leftPow;
        r = rightPow;
        motorFwdLeft.setPower(l);
        motorFwdRight.setPower(-r);
        motorBackLeft.setPower(-l);
        motorBackRight.setPower(r);
    }

    //turns on camera and starts gold detector so it can be used to find the x and y location of the gold
    public void enableGoldDetector(){
        goldDetector = new GoldAlignDetector();
        goldDetector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        goldDetector.useDefaults();
        // Optional Tuning used mostly in development
        goldDetector.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        goldDetector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
        goldDetector.downscale = 0; // How much to downscale the input frames
        goldDetector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        goldDetector.maxAreaScorer.weight = 0.005;
        goldDetector.ratioScorer.weight = 5;
        goldDetector.ratioScorer.perfectRatio = 0;
        goldDetector.enable();
    }

    //turns on camera and starts order detector - scans for both of the silver and the gold at the same time and returns
    //                                            the location of the gold relative to the two silver
    //### not currently used as requires all three minerals to be in camera view which our camera cannot do
    public void enableOrderDetector(){
        orderDetector = new SamplingOrderDetector();
        orderDetector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        orderDetector.useDefaults();

        orderDetector.downscale = 0; // How much to downscale the input frames

        // Optional Tuning
        orderDetector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        orderDetector.maxAreaScorer.weight = 0.005;

        orderDetector.ratioScorer.weight = 5;
        orderDetector.ratioScorer.perfectRatio = 0;

        orderDetector.enable();
    }

    //  Pause for the specified amount of time (time: mili secs)
    public void pause(long millis){
        runtime.reset();
        while (opModeIsActive() && runtime.milliseconds() <= millis){

        }
    }
    //Using the gyroscope, when a degree is passed both left and right motors move accordingly in
    //order to turn the robot to the right or left until the bearing is equal to or greater than the
    //specified degree. Power can also be specified. It acts like a turnTo() with a built in
    //resetAngle function. Cannot be used without resetting the angle.
    //used commonly in pairs(one fast for speed and one slow for accuracy) to improve movement time.
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
    //Using the gyroscope, when a degree is passed both left and right motors move accordingly in
    //order to turn the robot to the right or left until the bearing is equal to or greater than the
    //specified degree. Power can also be specified.
    //used commonly in pairs(one fast for speed and one slow for accuracy) to improve movement time.
    public void turnToS(int degrees, double power){
        int left = 1;
        int right = 1;
        double distance = getAngle() - degrees;
        double startAngle = getAngle();
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
                if((startAngle + ((distance/4)*3)) > getAngle()){
                    left *= 1.05;
                    right *= 1.05;
                }
                else{
                    if(left > 1 || left < -1 || right > 1 || right < -1){
                        left*=0.95;
                        right*=0.95;
                    }
                }
            }
        }
        else {    // right turn.
            while (opModeIsActive() && getAngle() <= degrees) {
                telemetry.addData("Angle4", getAngle());
                telemetry.update();
                if((startAngle + ((distance/4)*3)) > getAngle()){
                    left *= 1.01;
                    right *= 1.01;
                }
                else{
                    if(left > 1 || left < -1 || right > 1 || right < -1){
                        left*=0.99;
                        right*=0.99;
                    }
                }
            }
        }
        // turn the motors off.
        motorBackLeft.setPower(0);
        motorFwdLeft.setPower(0);
        motorBackRight.setPower(0);
        motorFwdRight.setPower(0);
    }    //Using the gyroscope, when a degree is passed both left and right motors move accordingly in
    //order to turn the robot to the right or left until the bearing is equal to or greater than the
    //specified degree. Power can also be specified.
    //used commonly in pairs(one fast for speed and one slow for accuracy) to improve movement time.
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
    //Resets gyro sensor bearing value to 0
    //commonly used to calibrate before a match as well
    public void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }
    //Resets gyro sensor pitch value to 0
    //commonly used to calibrate before a match as well
    public void resetPitch()
    {
        lastPitches = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalPitch = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    //uses the imu to find the current angle
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
    //uses the imu to get the current pitch of the robot
    public double getPitch()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.secondAngle - lastPitches.secondAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalPitch += deltaAngle;

        lastPitches = angles;

        return globalPitch;
    }


    //drives the robot in a straight line using the gyro sensor and the distance sensor
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

    //  Initiates all hardware for autonomous programs
    public void initForAutonomous()
    {
        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        BNO055IMU.Parameters gyroParameters = new BNO055IMU.Parameters();

        gyroParameters.mode                = BNO055IMU.SensorMode.IMU;
        gyroParameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        gyroParameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        gyroParameters.loggingEnabled      = false;

        //initialize hardware
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
        distanceSensorL = hardwareMap.get(Rev2mDistanceSensor.class, "distanceSensorL");
        distanceSensorR = hardwareMap.get(Rev2mDistanceSensor.class, "distanceSensorR");
        lights = hardwareMap.get(RevBlinkinLedDriver.class, "lights");

        //lights turn green to show that hardware is initialized
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);

        //initializes imu and calibrates it. Prepares lift motor to land using the encoder
        // Lights turn green when it is calibrated
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
