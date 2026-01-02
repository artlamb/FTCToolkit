package competition;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import common.Auto;
import common.Logger;

@Disabled
@Autonomous(name="Blue Audience", group="Competition")
@com.acmerobotics.dashboard.config.Config

public class AutoBlueAudience extends LinearOpMode {

    public static long delay = 0;
    public static double launcherSpeed = 28;
    public static Auto.Alliance alliance = Auto.Alliance.BLUE;
    public static Auto.StartPosition startPosition = Auto.StartPosition.AUDIENCE;
    public Auto.Order[] order = { Auto.Order.TOP, Auto.Order.MIDDLE, Auto.Order.BOTTOM };

    @Override
    public void runOpMode() {

        try {
            Auto auto = new common.Auto(this, alliance, startPosition, order, launcherSpeed, delay);
            auto.runAuto();

        } catch (Exception e) {
            Logger.error(e, "Exception");
            throw e;
        }
    }
}
