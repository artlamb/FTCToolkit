/**
 * This class contains methods to log messages to Android Studio's Logcat window.
 */

package common;

@com.acmerobotics.dashboard.config.Config

public final class Debug {

    public static boolean drive = true;
    public static boolean launcher = true;
    public static boolean dashboard = true;
    public static boolean drawPaths = false;
    public static boolean waitForButtonPress = false;


    public static boolean drive() {
        return drive;
    }

    public static void setDrive(boolean value) {
        drive = value;
    }

    public static boolean launcher() {
        return launcher;
    }

    public static void setLauncher(boolean value) {
        launcher = value;
    }

    public static boolean dashboard() {
        return dashboard;
    }

    public static void setDashboard(boolean value) {
        dashboard = value;
    }

    public static boolean drawPaths() {
        return drawPaths;
    }

    public static void setDrawPaths(boolean value) {
        drawPaths = value;
    }

    public static boolean waitForButtonPress() {
        return waitForButtonPress;
    }

    public static void setEnableWait(boolean value) {
        waitForButtonPress = value;
    }
}
