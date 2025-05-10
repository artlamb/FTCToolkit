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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.Locale;

import common.Drive;
import common.DriveControl;
import common.DriveGamepad;
import common.Logger;
import drivers.GoBildaPinpointDriver;
import utils.Pose;

@TeleOp(name="DriveToTest", group="Test")
//@SuppressLint("DefaultLocale")
@com.acmerobotics.dashboard.config.Config

public class DriveToTest extends LinearOpMode {

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

            driveToCoordinateTest();

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

    private void driveToCoordinateTest() {

        double x = 20;
        double y = 20;
        double startHeading = 90;
        double targetHeading = 90;

        driveControl.setPose(new Pose(0, 0, Math.toRadians(startHeading)));

        while (opModeIsActive()) {

            if (gamepad1.y) {
                driveControl.moveToCoordinate(0, y, targetHeading,2000);
                while (gamepad1.y) sleep(10);
                sleep(1000);
                displayPose();
            }

            if (gamepad1.a) {
                driveControl.moveToCoordinate(0, -y, targetHeading,2000);
                while (gamepad1.a) sleep(10);
                sleep(1000);
                displayPose();
            }

            if (gamepad1.b) {
                driveControl.moveToCoordinate(x, 0, targetHeading,2000);
                while (gamepad1.b) sleep(10);
                sleep(1000);
                displayPose();
            }

            if (gamepad1.x) {
                driveControl.moveToCoordinate(-x, 0,targetHeading,2000);
                sleep(1000);
                displayPose();
                while (gamepad1.x) sleep(10);
            }

            if (gamepad1.right_bumper) {
                driveControl.moveToCoordinate(0, 0, startHeading,2000);
                sleep(1000);
                displayPose();
                while (gamepad1.right_bumper) sleep(10);

            }
            displayPose(true);
        }
    }


    private void displayPose() {
        displayPose(false);
    }

    private void displayPose(boolean telemetryOnly) {

        Pose pose = driveControl.getPose();
        pinpoint.update();

        String str1 = String.format(Locale.US, "x %5.1f  y %5.1f  heading %5.1f", pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));
        String str2 = String.format(Locale.US, "x %d  y %d", pinpoint.getEncoderX(), pinpoint.getEncoderY());

        telemetry.addData("pose", str1);
        telemetry.addData("encoders", str2);
        telemetry.update();

        if (! telemetryOnly) {
            Logger.message("pose: %s", str1);
        }
    }
}
