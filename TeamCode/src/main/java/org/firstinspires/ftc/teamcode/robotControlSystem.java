/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorController;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 *
 * Updated 12/2/2016
 */

@TeleOp(name="RobotControlSystem", group="Iterative Opmode")  // @AutonomousSense(...) is the other common choice
@Disabled
public class robotControlSystem extends OpMode
{
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

     private DcMotor leftMotor = null;
     private DcMotor leftMotor2 = null;
     private DcMotor rightMotor = null;
     private DcMotor rightMotor2 = null;
     private DcMotor catapultMotor = null;
     private DcMotor intake = null;

    final static int ENCODER_CPR = 1120;    //Encoder counts per Revolution
    int degrees = 180; //sets the degrees we want the motor to turn
    double counts = (double) degrees * ENCODER_CPR/360.0; //sets the amount of counts for the motor to turn to tun the spesified derees

    //variables for intake
    double iPwr = 1; //power for intake


    //functions for The Dominator (Robot 2)
    public void leftMotors(float pow){
        leftMotor2.setPower(-pow);
        leftMotor.setPower(-pow);
    }

    public void  rightMotors(float pow){
        rightMotor2.setPower(-pow);
        rightMotor.setPower(-pow);
    }

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        //telemetry.addData("Status", "Initialized");

        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */
         leftMotor  = hardwareMap.dcMotor.get("left_drive");
         leftMotor2 = hardwareMap.dcMotor.get("left_drive2");
         rightMotor = hardwareMap.dcMotor.get("right_drive");
         rightMotor2 = hardwareMap.dcMotor.get("right_drive2");
         //catapultMotor = hardwareMap.dcMotor.get("cat");
         //intake = hardwareMap.dcMotor.get("intake");

        // eg: Set the drive motor directions:
        // Reverse the motor that runs backwards when connected directly to the battery
        // leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
          rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
          rightMotor2.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
          //catapultMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
          /*catapultMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
          catapultMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
*/
          telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

        runtime.reset();

        /*catapultMotor.setTargetPosition((int) counts);
        catapultMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        catapultMotor.setPower(1);*/
    }
// comment
    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    boolean AButonOn = false;
    boolean BButonOn = false;
    @Override
    public void loop() {
        telemetry.addData("Status", "Running: " + runtime.toString());
        //left and right drive motors
        float leftY = -gamepad1.left_stick_y; //power for left_motor attached to left controller stick
        float rightY = -gamepad1.right_stick_y; //poewr for right_motor attached to right controller stick
        telemetry.addData("Left Gamepad",leftY);
        telemetry.addData("Right Gamepad",rightY);
        //intake
        boolean buttonA = gamepad1.a;
        boolean buttonB = gamepad1.b;
        //catapult
        boolean bumper1 = gamepad1.left_bumper;
        boolean bumper2 = gamepad1.right_bumper;
        boolean buttonY = gamepad1.y;
        boolean up = gamepad1.dpad_up;
        boolean down = gamepad1.dpad_down;
        boolean right = gamepad1.dpad_right;
        float triggerR = gamepad1.right_trigger;
        // eg: Run wheels in tank mode (note: The joystick goes negative when pushed forwards)
         leftMotor.setPower(leftY);   //got rid of original code to replace it with code that
         rightMotor.setPower(rightY); //works with the Dominater(robot 2)
        //leftMotors(leftY);
        //rightMotors(rightY);
        //Run the intake

        if(up){
            intake.setPower(iPwr);
            /*if(AButonOn){
                intake.setPower(0);
            }
           else{
                intake.setPower(iPwr);
                AButonOn = true;
            }*/

        }
        if(right){
            intake.setPower(0);
        }
        if(down){
            intake.setPower(-iPwr);
            /*if(BButonOn){
                intake.setPower(0);
            }
            else {
                intake.setPower(-iPwr);
                BButonOn = true;
            }*/
        }
        //Run the catapult
        /*if(up){
            //catapultMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            catapultMotor.setPower(.2);
        }
        if(down){
            //catapultMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            catapultMotor.setPower(0);
        }*/


        //catapultMotor.setPower(triggerR);
        if(buttonA){
            catapultMotor.setPower(.5);
        }
        if(bumper1){
            catapultMotor.setPower(0);
        }
        if(bumper2){
            catapultMotor.setPower(1);
        }

        //telemetry.addData("Motor Target", counts);
        //telemetry.addData(/*Left*/"Position", catapultMotor.getCurrentPosition());
}

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
