/**
 * This class contains methods to log messages to Android Studio's Logcat window.
 */

package common;

import android.util.Log;

import java.util.Objects;
@com.acmerobotics.dashboard.config.Config

public final class Logger {
    public static int errorLevel = Log.VERBOSE;
    private static final String TAG = "DELMAR";

    private static String format(String str) {
        String caller = Thread.currentThread().getStackTrace()[4].getMethodName();
        return String.format("%-16s %-24s %s", Thread.currentThread().getName(), caller, str);
    }

    public static void message(String msg) {
        String str = format(msg);
        Log.d(TAG, str);
    }

    public static void message(String format, Object... args){
        String str = format(String.format(format, args));
        Log.d(TAG, str);
    }

    public static void info(String format, Object... args) {
        if (errorLevel <= Log.INFO) {
            String str = format(String.format(format, args));
            Log.i(TAG, str);
        }
    }

    public static void debug(String format, Object... args){
        if (errorLevel <= Log.DEBUG) {
            String str = String.format(format, args);
            Log.d(TAG, str);
        }
    }

    public static void verbose(String format, Object... args) {
        if (errorLevel <= Log.VERBOSE) {
            String str = format(String.format(format, args));
            Log.v(TAG, str);
        }
    }

    public static void warning(String warning) {
        String str = format(warning);
        Log.w(TAG, str);
    }

    public static void warning(String format, Object... args) {
        String str = format(String.format(format, args));
        Log.w(TAG, str);
    }

    public static void error (Exception e, String msg) {
        error(e, msg, 10);
    }

    public static void error (Exception e, String msg, int traceDepth) {

        Log.e(TAG, "\n");
        Log.println(Log.ERROR, TAG, msg);
        Log.e(TAG, Objects.requireNonNull(e.getMessage()));

        if (traceDepth > 0 ) {
            StackTraceElement[] stackTraceElements = e.getStackTrace();
            for (int i=0; i<Math.min(stackTraceElements.length, traceDepth); i++)
                Log.e(TAG, stackTraceElements[i].toString());
        }
    }

    public static void addLine(String msg) {
        Log.d(TAG, msg);
    }

    public static void setErrorLevel(int level) {
        errorLevel = level;
    }

}
