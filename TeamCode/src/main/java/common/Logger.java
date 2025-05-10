/**
 * This class contains methods to log messages to Android Studio's Logcat window.
 */

package common;

import android.util.Log;

import java.util.Objects;

public final class Logger {
    public static final boolean VERBOSE = true;
    private static final String TAG = "DELMAR";

    public static String getCaller () {
        String caller = Thread.currentThread().getStackTrace()[4].getMethodName();
        return String.format("%-16s", caller);
    }

    private static String format(String str) {
        String caller = Thread.currentThread().getStackTrace()[5].getMethodName();
        return String.format("%-20s %-24s %s", Thread.currentThread().getName(), caller, str);
    }

    private static void logString (String message) {
        String str = format(message);
        Log.d(TAG, str);
    }

    public static void warning(String warning) {
        String str = format(warning);
        Log.w(TAG, str);
    }

    public static void warning(String format, Object... args) {
        String str = format(String.format(format, args));
        Log.w(TAG, str);
    }

    public static void message(String msg) {
        logString(msg);
    }

    public static void message(String format, Object... args){
        logString(String.format(format, args));
    }

    public static void verbose(String format, Object... args) {
        if (VERBOSE) {
            logString(String.format(format, args));
        }
    }

    public static void error (Exception e, String msg) {
        error(e, msg, true);
    }

    public static void error (Exception e, String msg, boolean stacktrace) {

        Log.e(TAG, "\n");
        Log.println(Log.ERROR, TAG, msg);
        Log.e(TAG, Objects.requireNonNull(e.getMessage()));

        if (stacktrace) {
            StackTraceElement[] stackTraceElements = e.getStackTrace();
            for (int i=0; i<Math.min(stackTraceElements.length, 10); i++)
                Log.e(TAG, stackTraceElements[i].toString());
        }
    }

    public static void addLine(String msg) {
        Log.d(TAG, msg);
    }

}
