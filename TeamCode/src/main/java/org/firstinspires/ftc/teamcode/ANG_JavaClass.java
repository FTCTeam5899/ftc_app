package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


    @TeleOp
    public class MyFIRSTJavaOpMode extends LinearOpMode {
        private Gyroscope imu;
        private DcMotor Andy;
        private DigitalChannel digitalTouch;
        private DistanceSensor Carly;
        private Servo Seral;
        @Override
        public void runOpMode() {
            imu = hardwareMap.get(Gyroscope.class, "imu");
            Andy = hardwareMap.get(DcMotor.class, "Andy");
            digitalTouch = hardwareMap.get(DigitalChannel.class, "digitalTouch");
            Carly = hardwareMap.get(DistanceSensor.class, "Carly");
            Seral = hardwareMap.get(Servo.class, "Seral");
            telemetry.addData("Status", "Initialized");
            telemetry.update();
            // Wait for the game to start (driver presses PLAY)
            waitForStart();
            // run until the end of the match (driver presses STOP)
            while (opModeIsActive()) {
                telemetry.addData("Status", "Running");
                telemetry.update();
            }
        }
    }












}
