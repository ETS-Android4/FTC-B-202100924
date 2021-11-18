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


@TeleOp(name="Reset Hardwares", group="Reset")
public class ResetHardwares extends OpMode
{
    // Elapsed time
    private ElapsedTime runtime = new ElapsedTime();

    // Declaring wheel motors
    private DcMotor leftFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightFront = null;
    private DcMotor rightBack = null;

    // Declaring other motor
    private DcMotor liftMotor = null;
    private DcMotor spinnerMotor = null;

    // Declaring servo
    private Servo wristL = null;
    private Servo wristR = null;


    @Override
    public void init() {
        // Initializing message
        telemetry.addData("Status", "Resetting");

        // Initializing wheel motors variable
        leftFront  = hardwareMap.get(DcMotor.class, "left_front"); // control
        leftBack = hardwareMap.get(DcMotor.class, "left_back"); // extension
        rightFront = hardwareMap.get(DcMotor.class, "right_front"); // control
        rightBack = hardwareMap.get(DcMotor.class, "right_back"); // extension

        // Configuring directions of the motors
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        // Initializing lift motor and configuring brake behavior
        liftMotor = hardwareMap.get(DcMotor.class, "lift_motor"); // extension
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setPower(1.0);
        liftMotor.setTargetPosition(0);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Initializing spinner motor
        wristL = hardwareMap.get(Servo.class, "wristL"); // control
        wristR = hardwareMap.get(Servo.class, "wristR"); // control
        wristL.setDirection(Servo.Direction.REVERSE);

        // Initializing spinner motor and configuring direction
        spinnerMotor = hardwareMap.get(DcMotor.class, "spinner_motor"); // extension
        spinnerMotor.setDirection(DcMotor.Direction.REVERSE);

        // Ready message
        telemetry.addData("Status", "Finished resetting");
    }


    @Override
    public void init_loop() {

    }


    @Override
    public void loop() {

        if (gamepad2.a) {
            wristL.setPosition(1.0);
            wristR.setPosition(0.9);
        }
        else if (gamepad2.b) {
            wristL.setPosition(-1.0);
            wristR.setPosition(-1.0);
        }

    }


    @Override
    public void stop() {
        telemetry.addData("Status", "Stopped");
    }

}