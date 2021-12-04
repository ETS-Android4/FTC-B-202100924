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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(group="TeleOp")
public class MainTeleOpNoLimit extends OpMode
{
    // Elapsed time
    private ElapsedTime runtime = new ElapsedTime();

    // Declaring motors
    private DcMotor leftFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightFront = null;
    private DcMotor rightBack = null;
    private DcMotor liftMotor = null;
    private DcMotor spinnerMotor = null;

    // Declaring servos
    private Servo wristL = null;
    private Servo wristR = null;


    @Override
    public void init() {

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

        // Initializing spinner motor
        wristL = hardwareMap.get(Servo.class, "wristL");
        wristR = hardwareMap.get(Servo.class, "wristR");
        wristL.setDirection(Servo.Direction.REVERSE);

        // Initializing spinner motor and configuring direction
        spinnerMotor = hardwareMap.get(DcMotor.class, "spinner_motor");
        spinnerMotor.setDirection(DcMotor.Direction.REVERSE);

    }


    private void movement(double frontL, double backL, double frontR, double backR) {
        leftFront.setPower(frontL);
        leftBack.setPower(backL);
        rightFront.setPower(frontR);
        rightBack.setPower(backR);
    }

    private void move_lift(int target) {
        liftMotor.setPower(1.0);
        liftMotor.setTargetPosition(target);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    @Override
    public void loop() {

        // Declaring power level of wheel motors
        double leftPower = -gamepad1.left_stick_y * 0.5;
        double rightPower = -gamepad1.right_stick_y * 0.5;

        if (runtime.seconds() > 0.0) {

            // Send power level to wheel motors
            if ((gamepad1.left_bumper) && (!gamepad1.dpad_down) && (!gamepad1.dpad_up) && (gamepad1.left_trigger < 0.25) && (gamepad1.right_trigger < 0.25)) {
                movement(-leftPower, leftPower, leftPower, -leftPower);
            } else if ((gamepad1.right_bumper) && (!gamepad1.dpad_down) && (!gamepad1.dpad_up) && (gamepad1.left_trigger < 0.25) && (gamepad1.right_trigger < 0.25)) {
                movement(rightPower, -rightPower, -rightPower, rightPower);
            } else if ((!gamepad1.dpad_down) && (!gamepad1.dpad_up) && (gamepad1.left_trigger < 0.25) && (gamepad1.right_trigger < 0.25)) {
                movement(leftPower, leftPower, rightPower, rightPower);
            } else if ((gamepad1.dpad_up) && (gamepad1.left_trigger < 0.25) && (gamepad1.right_trigger < 0.25)) {
                movement(0.80, 0.80, 0.80, 0.80);
            } else if ((gamepad1.dpad_down) && (gamepad1.left_trigger < 0.25) && (gamepad1.right_trigger < 0.25)) {
                movement(-0.80, -0.80, -0.80, -0.80);
            } else if ((gamepad1.left_trigger > 0.25) && (gamepad1.right_trigger < 0.25)) {
                movement(-0.75, 0.75, 0.75, -0.75);
            } else if ((gamepad1.left_trigger < 0.25) && (gamepad1.right_trigger > 0.25)) {
                movement(0.75, -0.75, -0.75, 0.75);
            }

            // Up and down control for lift motor
            if (!liftMotor.isBusy()) {
                if (gamepad2.dpad_up) {
                    move_lift(4800);
                } else if (gamepad2.dpad_down) {
                    move_lift(0);
                }
            }

            // Open and close wrist (servo)
            if (gamepad2.a) {
                wristL.setPosition(1.0);
                wristR.setPosition(0.9);
            } else if (gamepad2.b) {
                wristL.setPosition(-1.0);
                wristR.setPosition(-1.0);
            }

            // Spin spinner motor
            if (gamepad2.y) {
                spinnerMotor.setPower(0.2);
            } else {
                spinnerMotor.setPower(0.0);
            }

            // Update status
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors' Power Level", "Left: (%.2f), Right: (%.2f)", ((leftPower * 100) + '%'), ((rightPower * 100) + '%'));
            telemetry.addData("Lift's Position", "Pos. Value: " + liftMotor.getCurrentPosition());
        }

    }

    @Override
    public void stop() {
        telemetry.addData("Status", "Stopped");
    }

}