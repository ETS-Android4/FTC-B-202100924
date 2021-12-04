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


@Autonomous(group="Auto")
public class AutoTemp extends LinearOpMode {

    /* Declare OpMode members. */
    Hardwares robot   = new Hardwares();
    private ElapsedTime runtime = new ElapsedTime();


    private void movement(double frontL, double backL, double frontR, double backR) {
        robot.leftFront.setPower(frontL);
        robot.leftBack.setPower(backL);
        robot.rightFront.setPower(frontR);
        robot.rightBack.setPower(backR);
    }

    private void movement_wait(int time, String name) {
        while (opModeIsActive() && (runtime.seconds() < time)) {
            telemetry.addData(name, "%2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
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

        // init
        robot.wristL.setPosition(-1.0);
        robot.wristR.setPosition(-1.0);
        runtime.reset();
        movement_wait(1, "Going to center");

        // 1st
        runtime.reset();
        robot.liftMotor.setPower(1.0);
        robot.liftMotor.setTargetPosition(4800);
        robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (opModeIsActive() && robot.liftMotor.isBusy()) {
            telemetry.addData("Lifting", "%2.5f S Elapsed", runtime.seconds());
            telemetry.addData("Lift", "Pos. Value: " + robot.liftMotor.getCurrentPosition());
            telemetry.update();
        }
        robot.liftMotor.setPower(0.0);


        // 2nd
        movement(0.4, 0.4, 0.4, 0.4);
        runtime.reset();
        movement_wait(1, "Going to deposit");
        movement(0.0, 0.0, 0.0, 0.0);
        sleep(100);
        robot.wristL.setPosition(1.0);
        robot.wristR.setPosition(0.9);


        // 3rd
        runtime.reset();
        movement(-0.4, 0.5, 0.4, -0.4);
        movement_wait(2, "Going to the wall");
        movement(0.0, 0.0, 0.0, 0.0);


        // 4th
        runtime.reset();
        movement(-0.41, 0.61, 0.41, -0.41);
        movement_wait(2, "Going to the wall");
        movement(0.0, 0.0, 0.0, 0.0);

        // 5th
        runtime.reset();
        movement(-0.25, -0.25, -0.25, -0.25);
        movement_wait(2, "Going to spin");
        movement(0.0, 0.0, 0.0, 0.0);


        // 6th
        robot.spinnerMotor.setPower(0.3);
        runtime.reset();
        movement_wait(8, "Spinning");
        robot.spinnerMotor.setPower(0.0);

        // 7th
        runtime.reset();
        movement(0.35, 0.35, 0.35, 0.35);
        movement_wait(1, "Going to park");
        movement(0.0, 0.0, 0.0, 0.0);

        // 8th
        runtime.reset();
        movement(0.2, -0.2, -0.2, 0.2);
        movement_wait(1, "Going to park");
        movement(0.0, 0.0, 0.0, 0.0);


        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }
}
