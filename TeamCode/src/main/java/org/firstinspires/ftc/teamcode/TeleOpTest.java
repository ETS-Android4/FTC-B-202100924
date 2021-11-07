/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="uwu test", group="Mecanum")
public class TeleOpTest extends OpMode
{
    // Elapsed time
    private ElapsedTime runtime = new ElapsedTime();

    // Declaring wheel motors
    private DcMotor leftFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightFront = null;
    private DcMotor rightBack = null;

    // Declaring lift motor
    private DcMotor liftMotor = null;

    // Declaring servo
    private Servo wristL = null;
    private Servo wristR = null;

    // Declaring spinner motor
    private DcMotor spinnerMotor = null;

    // Other variables
    private double rightPower = 0.0;
    private double leftPower = 0.0;
    private double spinnerPower = 0.0;
    private double powerMultiplier = 0.75;

    private final int initializingID = hardwareMap.appContext.getResources().getIdentifier("initializing", "raw", hardwareMap.appContext.getPackageName());
    private final int onehalfID = hardwareMap.appContext.getResources().getIdentifier("onehalf", "raw", hardwareMap.appContext.getPackageName());
    private final int oneID = hardwareMap.appContext.getResources().getIdentifier("one", "raw", hardwareMap.appContext.getPackageName());
    private final int halfID = hardwareMap.appContext.getResources().getIdentifier("half", "raw", hardwareMap.appContext.getPackageName());
    private final int zeroID = hardwareMap.appContext.getResources().getIdentifier("zero", "raw", hardwareMap.appContext.getPackageName());
    private int soundPhase = 0;
    private double timePhase = 30.0;


    @Override
    public void init() {
        // Initializing message
        telemetry.addData("Status", "Initializing");
        SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, initializingID);

        // Initializing wheel motors variable
        leftFront  = hardwareMap.get(DcMotor.class, "left_front");
        leftBack = hardwareMap.get(DcMotor.class, "left_back");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        rightBack = hardwareMap.get(DcMotor.class, "right_back");

        // Configuring directions of the motors
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        // Initializing lift motor and configuring brake behavior
        liftMotor = hardwareMap.get(DcMotor.class, "lift_motor");
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setPower(1.0);
        liftMotor.setTargetPosition(0);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Initializing spinner motor
        wristL = hardwareMap.get(Servo.class, "wristL");
        wristR = hardwareMap.get(Servo.class, "wristR");

        // Initializing spinner motor and configuring direction
        spinnerMotor = hardwareMap.get(DcMotor.class, "spinner_motor");
        spinnerMotor.setDirection(DcMotor.Direction.REVERSE);

        // Ready message
        telemetry.addData("Status", "Initialized");
    }


    @Override
    public void init_loop() {

        // Logics for power multiplier
        if (gamepad1.dpad_down) {
            powerMultiplier = 0.5;
        } else if (gamepad1.dpad_up) {
            powerMultiplier = 0.75;
        }

        // Logics for power
        if (runtime.seconds() < 120.0) {
            rightPower = -gamepad1.right_stick_y * powerMultiplier;
            leftPower = -gamepad1.left_stick_y * powerMultiplier;
            spinnerPower = 0.2;
        }
        else {
            rightPower = 0.0;
            leftPower = 0.0;
            spinnerPower = 0.0;
        }

        // Logics for time remainder
        if (runtime.seconds() > timePhase) {
            if (soundPhase == 0) {
                timePhase += 30.0;
                soundPhase += 1;
                SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, onehalfID);
            } else if (soundPhase == 1) {
                timePhase += 30.0;
                soundPhase += 1;
                SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, oneID);
            } else if (soundPhase == 2) {
                timePhase += 30.0;
                soundPhase += 1;
                SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, halfID);
            } else if (soundPhase == 3) {
                timePhase += 30.0;
                soundPhase += 1;
                SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, zeroID);
            }
        }

    }


    @Override
    public void loop() {

        // Send power level to wheel motors
        if (gamepad1.left_bumper) {
            leftFront.setPower(-leftPower);
            leftBack.setPower(leftPower);
            rightFront.setPower(leftPower);
            rightBack.setPower(-leftPower);
        } else if (gamepad1.right_bumper) {
            leftFront.setPower(rightPower);
            leftBack.setPower(-rightPower);
            rightFront.setPower(-rightPower);
            rightBack.setPower(rightPower);
        } else {
            leftFront.setPower(leftPower);
            leftBack.setPower(leftPower);
            rightFront.setPower(rightPower);
            rightBack.setPower(rightPower);
        }

        // Up and down control for lift motor
        if (!liftMotor.isBusy()) {
            if (gamepad2.dpad_up) {
                liftMotor.setPower(1.0);
                liftMotor.setTargetPosition(5450);
                liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            else if (gamepad2.dpad_down) {
                liftMotor.setPower(1.0);
                liftMotor.setTargetPosition(0);
                liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
        }

        // Open and close wrist (servo)
        if (gamepad2.a) {
            wristL.setPosition(0.0);
            wristR.setPosition(0.5);
        }
        else if (gamepad2.b) {
            wristL.setPosition(0.5);
            wristR.setPosition(0.0);
        }

        // Spin spinner motor
        if (gamepad2.y) {
            spinnerMotor.setPower(spinnerPower);
        } else if (gamepad2.x) {
            spinnerMotor.setPower(-spinnerPower * 0.25);
        } else {
            spinnerMotor.setPower(0.0);
        }

        // Update status
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors' Power Level", "Left: (%.2f), Right: (%.2f)", leftPower, rightPower);
        telemetry.addData("Spinner Power Level", "Power Value: " + spinnerPower);

        if (liftMotor.isBusy()) {
            telemetry.addData("Lift Status", "Status: Busy, Pos. Value: " + liftMotor.getCurrentPosition());
        } else {
            telemetry.addData("Lift Status", "Status: Idle, Pos. Value: " + liftMotor.getCurrentPosition());
        }

    }


    @Override
    public void stop() {
        telemetry.addData("Status", "Stopped");
    }

}