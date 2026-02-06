package test;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import common.Config;
import common.DriveControl;
import common.Logger;
import common.Robot;
import utils.Pose;

@TeleOp(name="Drivetrain Test", group="Test")
@SuppressLint("DefaultLocale")
@com.acmerobotics.dashboard.config.Config

public class DriveTrainTest extends LinearOpMode {

    public static double power = 0.20;

    Robot robot;

    @Override
    public void runOpMode() {
        try {
            robot = new Robot(this);
            DcMotorEx leftFront = hardwareMap.get(DcMotorEx.class, Config.LEFT_FRONT);
            DcMotorEx rightFront = hardwareMap.get(DcMotorEx.class, Config.RIGHT_FRONT);
            DcMotorEx leftBack = hardwareMap.get(DcMotorEx.class, Config.LEFT_BACK);
            DcMotorEx rightBack = hardwareMap.get(DcMotorEx.class, Config.RIGHT_BACK);

            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            telemetry.addLine("Press start");
            telemetry.update();

            waitForStart();


            while (opModeIsActive()) {

                if (gamepad1.aWasPressed()) {
                    leftFront.setPower(power);
                }
                if (gamepad1.bWasPressed()) {
                    rightFront.setPower(power);
                }
                if (gamepad1.xWasPressed()) {
                    leftBack.setPower(power);
                }
                if (gamepad1.yWasPressed()) {
                    rightBack.setPower(power);
                }

                if (gamepad1.aWasReleased()) {
                    leftFront.setPower(0);
                }
                if (gamepad1.bWasReleased()) {
                    rightFront.setPower(0);
                }
                if (gamepad1.xWasReleased()) {
                    leftBack.setPower(0);
                }
                if (gamepad1.yWasReleased()) {
                    rightBack.setPower(0);
                }
            }

        } catch (Exception e) {
            Logger.error(e, "Error");
        }
    }


}
