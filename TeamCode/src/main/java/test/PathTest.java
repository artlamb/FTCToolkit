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

package test;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import drivers.GoBildaPinpointDriver;
import utils.Pose;

import common.Drive;
import common.DriveControl;
import common.DriveGamepad;
import common.Logger;

@Disabled
@TeleOp(name="Path Test", group="Test")
@SuppressLint("DefaultLocale")
@com.acmerobotics.dashboard.config.Config

public class PathTest extends LinearOpMode {

    private DriveGamepad driveGamepad;
    private DriveControl driveControl;
    private GoBildaPinpointDriver pinpoint;

    @Override
    public void runOpMode() {

        try {
            initialize();
            driveControl.start();
            driveGamepad.start();

            telemetry.addLine("Press start");
            telemetry.update();
            waitForStart();

            competitionTest1();

        } catch (Exception e) {
            Logger.error(e, "Exception");
            throw e;
        }
    }

    private void initialize() {
        driveControl = new DriveControl(this, new Drive(this));
        driveGamepad = new DriveGamepad(this, driveControl);
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
    }

    public static double START_X = 8.5;
    public static double START_Y = 102.5;
    public static double START_HEADING = 90;

    public static double BUCKET_X = 15.5;
    public static double BUCKET_Y = 126.5;
    public static double BUCKET_HEADING = 135;

    public static double YELLOW_RIGHT_X = 33;
    public static double YELLOW_RIGHT_Y = 120;
    public static double YELLOW_RIGHT_HEADING = 0;

    public static double YELLOW_MIDDLE_X = 33;
    public static double YELLOW_MIDDLE_Y = 130;
    public static double YELLOW_MIDDLE_HEADING = 0;

    public static double YELLOW_LEFT_X = 47;
    public static double YELLOW_LEFT_Y = 127.5;
    public static double YELLOW_LEFT_HEADING = 90;

    public static double PARK_X = 63;
    public static double PARK_Y = 104;
    public static double PARK_HEADING = 270;

    public static double ASCENT_X = 63;
    public static double ASCENT_Y = 95.5 ;
    public static double ASCENT_HEADING = 270;

    private void competitionTest1() {

        double timeout = 5000;

        driveControl.setPose(new Pose(START_X, START_Y, Math.toRadians(START_HEADING)));

        while (opModeIsActive()) {

            if (gamepad1.a) {
                driveControl.moveToPose(BUCKET_X, BUCKET_Y, BUCKET_HEADING, timeout);
                while (gamepad1.a) sleep(10);
            }

            if (gamepad1.b) {
                driveControl.moveToPose(YELLOW_RIGHT_X, YELLOW_RIGHT_Y, YELLOW_RIGHT_HEADING, timeout);
                while (gamepad1.b) sleep(10);
            }

            if (gamepad1.x) {
                driveControl.moveToPose(YELLOW_MIDDLE_X, YELLOW_MIDDLE_Y, YELLOW_MIDDLE_HEADING, timeout);
                while (gamepad1.x) sleep(10);
            }

            if (gamepad1.y) {
                driveControl.moveToPose(YELLOW_LEFT_X, YELLOW_LEFT_Y, YELLOW_LEFT_HEADING, timeout);
                while (gamepad1.y) sleep(10);
            }

            if (gamepad1.dpad_up) {
                driveControl.moveToPose(PARK_X, PARK_Y, PARK_HEADING, timeout);
                while (gamepad1.dpad_up) sleep(10);
            }

            if (gamepad1.dpad_down) {
                driveControl.moveToPose(ASCENT_X, ASCENT_Y, ASCENT_HEADING, 0.2, timeout);
                while (gamepad1.dpad_down) sleep(10);
            }

            if (gamepad1.right_bumper) {
                driveControl.moveToPose(START_X, START_Y, START_HEADING, timeout);
                while (gamepad1.right_bumper) sleep(10);
            }

            if (gamepad1.left_bumper) {
                driveControl.alignInCorner();
                while (gamepad1.left_bumper) sleep(10);
            }

            displayPose();
        }
    }

    private void competitionTest2() {

        double timeout = 5000;
        long start = System.currentTimeMillis();

        driveControl.setPose(new Pose(START_X, START_Y, Math.toRadians(START_HEADING)));

        driveControl.moveToPose(BUCKET_X, BUCKET_Y, BUCKET_HEADING, timeout);
        waitUntilNotMoving();

        driveControl.moveToPose(YELLOW_RIGHT_X, YELLOW_RIGHT_Y, YELLOW_RIGHT_HEADING, timeout);
        waitUntilNotMoving();

        driveControl.moveToPose(BUCKET_X, BUCKET_Y, BUCKET_HEADING, timeout);
        waitUntilNotMoving();

        driveControl.moveToPose(YELLOW_MIDDLE_X, YELLOW_MIDDLE_Y, YELLOW_MIDDLE_HEADING, timeout);
        waitUntilNotMoving();

        driveControl.moveToPose(BUCKET_X, BUCKET_Y, BUCKET_HEADING, timeout);
        waitUntilNotMoving();

        driveControl.moveToPose(YELLOW_LEFT_X, YELLOW_LEFT_Y, YELLOW_LEFT_HEADING, timeout);
        waitUntilNotMoving();

        driveControl.moveToPose(BUCKET_X, BUCKET_Y, BUCKET_HEADING, timeout);
        waitUntilNotMoving();

        Logger.message(String.format("time: %,d milliseconds", System.currentTimeMillis() - start));
    }

    private void moveFromTo(double startX, double startY, double startHeading, double targetX, double targetY, double targetHeading) {
        double timeout = 5000;

        for (int i = 0; i < 10; i++) {
            driveControl.setPose(new Pose(startX, startY, Math.toRadians(startHeading)));
            driveControl.moveToPose(targetX, targetY, targetHeading, timeout);
            waitUntilNotMoving();
            sleep(1000);
            driveControl.moveToPose(startX, startY, startHeading, timeout);
            waitUntilNotMoving();
            sleep(1000);
        }
    }

    private void waitUntilNotMoving() {
        while (driveControl.isBusy() && opModeIsActive()) {
            sleep(1);
        }
        sleep(100);
    }

    private void displayPose() {

        Pose pose = driveControl.getPose();
        pinpoint.update();

        String str1 = String.format("x %5.1f  y %5.1f  heading %5.1f", pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));
        String str2 = String.format("x %d  y %d", pinpoint.getEncoderX(), pinpoint.getEncoderY());

        telemetry.addData("pose", str1);
        telemetry.addData("encoders", str2);
        telemetry.update();

        Logger.message("pose: %s", str1);
    }
}
