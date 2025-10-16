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

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;

import common.Navigate;
import common.Robot;
import utils.Pose;

import common.DriveControl;
import common.DriveGamepad;
import common.Logger;
import utils.PoseData;
import utils.Waypoint;

@TeleOp(name="Path Test", group="Test")
@SuppressLint("DefaultLocale")
@Config

public class PathTest extends LinearOpMode {
    public enum Mode { AUTO_PATHS, MANUAL, GAMEPAD }
    public enum Alliance { BLUE, RED }

    public static Mode MODE = Mode.AUTO_PATHS;
    public static Alliance ALLIANCE = Alliance.BLUE;
    public static boolean READ_POSES = true;
    public static boolean WRITE_POSES = false;
    public static boolean DRAW_ONLY = true;
    public static PoseData waypoint = new PoseData(0, 0, 0, Waypoint.UNKNOWN);

    public static PoseData[] waypoints = {
            new PoseData(0, 0,  0, Waypoint.START),
            new PoseData(0, 0,  0, Waypoint.SHOOT_1),
            /*
            new PoseData(0, 0,  0, Waypoint.PICKUP_1),
            new PoseData(0, 0,  0, Waypoint.PICKUP_1),
            new PoseData(0, 0,  0, Waypoint.SHOOT_2),
            new PoseData(0, 0,  0, Waypoint.PICKUP_2),
            new PoseData(0, 0,  0, Waypoint.PICKUP_2),
            new PoseData(0, 0,  0, Waypoint.SHOOT_3),
            new PoseData(0, 0,  0, Waypoint.PICKUP_3),
            new PoseData(0, 0,  0, Waypoint.PICKUP_3),
             */
            new PoseData(0, 0,  0, Waypoint.PARK)
    };

    private DriveControl driveControl;
    private DriveGamepad driveGamepad;
    private Navigate navigate;

    @Override
    public void runOpMode() {

        try {
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

            initialize();

            if (READ_POSES)
                readPoses();
            addPaths();

            if (MODE == Mode.GAMEPAD) {
                driveGamepad.start();
            }

            displayPoses();

            while (opModeInInit()) {
                displayPose();
                editWaypoint();
                sleep(500);
            }

            //waitForStart();

            switch (MODE) {
                case AUTO_PATHS:
                    followPaths();
                    break;
                case MANUAL:
                    manualDrive();
                    break;
                case GAMEPAD:
                    gamepadDrive();
                    break;
            }

            if (WRITE_POSES)
                writePoses();

            while (opModeIsActive()) {
                editWaypoint();
                sleep(10);
            }

        } catch (Exception e) {
            Logger.error(e, "Exception");
            throw e;
        }
    }

    private void initialize() {
        Robot robot = new Robot(this);
        driveControl = robot.getDriveControl();
        driveGamepad = new DriveGamepad(this, driveControl);
        navigate = new Navigate(this, driveControl);

    }

    private void editWaypoint () {

        if (waypoint.desc == Waypoint.UNKNOWN) return;

        for (PoseData data: waypoints) {
            if (data.desc == waypoint.desc) {
                if (data.x == waypoint.x && data.y == waypoint.y && data.h == waypoint.h) return;
                data.x = waypoint.x;
                data.y = waypoint.y;
                data.h = waypoint.h;
                displayPoses();
                Pose pose = createPose(data.x, data.y, data.h);
                navigate.setPath(data.desc.name(), pose);
                navigate.drawPaths();
                return;
            }
        }
    }

    private Pose createPose(double x, double y, double heading) {
        return new Pose(x, y, Math.toRadians(heading));
    }

    private void addPaths() {

        for (PoseData data: waypoints) {
            Pose pose = createPose(data.x, data.y, data.h);
            if (navigate.pathExists(data.desc.name())) {
                navigate.appendPose(data.desc.name(), pose);
            } else {
                navigate.addPath(data.desc.name(), pose);
            }
        }
    }

    private void followPaths() {

        if (DRAW_ONLY) return;

        navigate.setStartingPose(0);

        for (int index = 1; index < navigate.numberOfPaths(); index++) {
            navigate.followPath(index);
            waitUntilNotMoving();
        }
    }

    private void manualDrive() {

        navigate.setStartingPose(0);

        while (opModeIsActive()) {

            if (gamepad1.a) {
                navigate.followPath(0);
                while (gamepad1.a) sleep(10);
            }

            if (gamepad1.b) {
                navigate.followPath(1);
                while (gamepad1.b) sleep(10);
            }

            if (gamepad1.x) {
                navigate.followPath(2);
                while (gamepad1.x) sleep(10);
            }

            if (gamepad1.y) {
                navigate.followPath(3);
                while (gamepad1.y) sleep(10);
            }

            waitUntilNotMoving();
            displayPose();
        }
    }

    private void gamepadDrive() {
        while (opModeIsActive()) {
            Thread.yield();
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

        String str1 = String.format("x %5.1f  y %5.1f  heading %5.1f", pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));

        telemetry.addData("pose", str1);
        telemetry.update();

        Logger.message("pose: %s", str1);
    }

    private void displayPoses () {
        for (PoseData data: waypoints) {
            String str1 = String.format("x %5.1f  y %5.1f  heading %5.1f  %s", data.x, data.y, data.h, data.desc.name());
            Logger.message("pose: %s", str1);
            telemetry.addLine(str1);
        }
        telemetry.update();
    }

    private String getPosesDirectory () {
        return String.format("%s/%s", AppUtil.FIRST_FOLDER.getAbsolutePath(), "settings");
    }

    private void writePoses ()  {

        try {
            String dir = getPosesDirectory();
            String filePath = String.format("%s/%s.txt", dir, "PathTest");
            String backPath = String.format("%s/%s.txt", dir, "PathTest_Backup");

            File file = new File(filePath);
            Logger.message("%s exist is %b", filePath, file.exists());
            if (file.exists()) {
                boolean renamed = file.renameTo(new File(backPath));
                if (renamed)
                    Logger.message("%s renamed to %s", filePath, backPath);
            }

            Gson gson = new GsonBuilder().setPrettyPrinting().create();
            FileWriter f = new FileWriter(filePath, false);

            gson.toJson(waypoints, f);
            f.flush();
            f.close();

        } catch (Exception e) {
            Logger.error(e, "Error writing settings");
        }
    }

    private void readPoses() {

        String dir = getPosesDirectory();
        String filePath = String.format("%s/%s.txt", dir, "PathTest");

        try {
            BufferedReader br = new BufferedReader(new FileReader(filePath));
            PoseData[] data = new Gson().fromJson(br, PoseData[].class);
            for (PoseData d : data) {
                for (PoseData d2 : waypoints) {
                    if (d.desc == d2.desc) {
                        d2.x = d.x;
                        d2.y = d.y;
                        d2.h = d.h;
                        Logger.message("%s", d.desc);
                    }
                }
            }
            //waypoints = new Gson().fromJson(br, PoseData[].class);
        } catch (Exception e) {
            Logger.error(e, "Error reading settings");
        }
    }


}
