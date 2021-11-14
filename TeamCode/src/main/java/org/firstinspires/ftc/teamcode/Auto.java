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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;


@Autonomous(name="uwu auto", group="Auto")
public class Auto extends LinearOpMode {

    /* Declare OpMode members. */
    Hardwares robot   = new Hardwares();
    private ElapsedTime runtime = new ElapsedTime();


    static final double     FORWARD_SPEED = 0.6;
    static final double     TURN_SPEED    = 0.5;

    private void movement(double frontL, double backL, double frontR, double backR) {
        robot.leftFront.setPower(frontL);
        robot.leftBack.setPower(backL);
        robot.rightFront.setPower(frontR);
        robot.rightBack.setPower(backR);
    }

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        // 1st
        movement(1.0, 1.0, 1.0, 1.0);
        runtime.reset();
        while (opModeIsActive() && (robot.rightBack.getCurrentPosition() < 100)) {
            telemetry.addData("Going to deposit", "%2.5f S Elapsed", runtime.seconds());
            telemetry.addData("Encoder", "Pos. Value: " + robot.rightBack.getCurrentPosition());
            telemetry.update();
        }
        movement(0.0, 0.0, 0.0, 0.0);


        // 2nd
        runtime.reset();
        robot.liftMotor.setPower(1.0);
        robot.liftMotor.setTargetPosition(5450);
        robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (opModeIsActive() && robot.liftMotor.isBusy()) {
            telemetry.addData("Lifting", "%2.5f S Elapsed", runtime.seconds());
            telemetry.addData("Lift", "Pos. Value: " + robot.liftMotor.getCurrentPosition());
            telemetry.update();
        }
        robot.liftMotor.setPower(0.0);


        // 3rd
        runtime.reset();
        robot.wristL.setPosition(0.5);
        robot.wristR.setPosition(0.0);
        while (opModeIsActive() && (robot.wristL.getPosition() != 0.5) && (robot.wristR.getPosition() != 0.0)) {
            telemetry.addData("Releasing", "%2.5f S Elapsed", runtime.seconds());
            telemetry.addData("Lift Pos. Value: ", "Left: (%.2f) Right: (%.2f)", robot.wristL.getPosition(), robot.wristR.getPosition());
            telemetry.update();
        }




        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way
        /*
        robot.leftFront.setPower(-FORWARD_SPEED);
        robot.leftBack.setPower(-FORWARD_SPEED);
        robot.rightFront.setPower(-FORWARD_SPEED);
        robot.rightBack.setPower(-FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 3.0)) {
            telemetry.addData("Going back to spin", "%2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        robot.leftFront.setPower(0);
        robot.leftBack.setPower(0);
        robot.rightFront.setPower(0);
        robot.rightBack.setPower(0);


        robot.spinnerMotor.setPower(0.3);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 3.0)) {
            telemetry.addData("Spinning", "%2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        robot.spinnerMotor.setPower(0);


        robot.leftFront.setPower(FORWARD_SPEED);
        robot.leftBack.setPower(FORWARD_SPEED);
        robot.rightFront.setPower(FORWARD_SPEED);
        robot.rightBack.setPower(FORWARD_SPEED);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 6.0)) {
            telemetry.addData("Moving to park", "%2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        robot.leftFront.setPower(0);
        robot.leftBack.setPower(0);
        robot.rightFront.setPower(0);
        robot.rightBack.setPower(0);
        */


        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }
}
