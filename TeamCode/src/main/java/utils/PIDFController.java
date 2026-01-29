package utils;

import android.annotation.SuppressLint;

import androidx.annotation.NonNull;

import java.util.Locale;

/**
 * This is the PIDFController class. This class handles the running of PIDFs. PIDF stands for
 * proportional, integral, derivative, and feedforward. PIDFs take the error of a system as an input.
 * Coefficients multiply into the error, the integral of the error, the derivative of the error, and
 * a feedforward value. Then, these values are added up and returned. In this way, error in the
 * system is corrected.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/5/2024
 */
@SuppressLint("DefaultLocale")

public class PIDFController {


    private PIDFCoefficients coefficients;

    public double previousError;
    public double error;
    private double errorDecelerated;
    private double position;
    private double targetPosition;
    private double errorProportional;
    private double errorIntegral;
    private double errorDerivative;
    private double errorSteadyState;
    private double feedForwardInput;
    private double output;

    private long previousUpdateTimeNano;
    private long deltaTimeNano;

    boolean reset = false;

    /**
     * This creates a new PIDFController from a PIDFCoefficients.
     *
     * @param set the coefficients to use.
     */
    public PIDFController(PIDFCoefficients set) {
        setCoefficients(set);
        reset();
    }

    /**
     * This takes the current error and runs the PIDF on it.
     *
     * @return this returns the value of the PIDF from the current error.
     */
    public double runPIDF() {
        output = errorProportional * P() + errorDerivative * D() + errorIntegral * I() + F() + errorSteadyState * S();
        return output;
    }

    /**
     * As opposed to updating position against a target position, this just sets the error to some
     * specified value.
     *
     * @param error The error specified.
     */
    public void updateError(double error) {
        if (previousError == 0) {
            previousError = error;
        } else {
            previousError = this.error;
        }
        this.error = error;

        long nanoSeconds = System.nanoTime();
        deltaTimeNano = nanoSeconds - previousUpdateTimeNano;
        previousUpdateTimeNano = nanoSeconds;

        double deltaError = error - previousError;
        errorProportional = error;
        errorIntegral += error * (deltaTimeNano / Math.pow(10.0, 9));
        errorDerivative = (error - previousError) / (deltaTimeNano / Math.pow(10.0, 9));
        errorSteadyState = (Math.abs(deltaError) < coefficients.sError ? error : 0);
    }

    /**
     * As opposed to updating position against a target position, this just sets the error to some
     * specified value.
     *
     * @param error The error specified.
     */
    public void updateError(double error, double deceleration) {

        if (reset) {
            previousError = error;
            reset = false;
        } else {
            previousError = this.error;
        }
        this.error = error;
        double deltaError = error - previousError;

        // Adjust pid errors to compensate for deceleration from current velocities,
        // don't decelerate to zero while there is still an error.
        errorDecelerated = error - deceleration;
        double minError = error * 0.1;
        if (MathUtil.getSign(error) > 0) {
            errorDecelerated = Math.max(minError, errorDecelerated);
        } else {
            errorDecelerated = Math.min(minError, errorDecelerated);
        }

        long nanoSeconds = System.nanoTime();
        deltaTimeNano = nanoSeconds - previousUpdateTimeNano;
        previousUpdateTimeNano = nanoSeconds;

        errorProportional = MathUtil.pow(errorDecelerated, coefficients.pExponent);
        errorIntegral += error * (deltaTimeNano / Math.pow(10.0, 9));
        errorDerivative = deltaError / (deltaTimeNano / Math.pow(10.0, 9));

        if ((Math.abs(error) < coefficients.sThreshold &&
                Math.abs(deltaError) < coefficients.sError)) {
            errorSteadyState = MathUtil.pow(error, coefficients.sExponent);
        } else {
            errorSteadyState = 0;
        }
    }

    /**
     * This can be used to update the PIDF's current position when inputting a current position and
     * a target position to calculate error. This will update the error from the current position to
     * the target position specified.
     *
     * @param update This is the current position.
     * @noinspection unused
     */
    public void updatePosition(double update) {
        position = update;
        previousError = error;
        error = targetPosition - position;

        deltaTimeNano = System.nanoTime() - previousUpdateTimeNano;
        previousUpdateTimeNano = System.nanoTime();

        errorIntegral += error * (deltaTimeNano / Math.pow(10.0, 9));
        errorDerivative = (error - previousError) / (deltaTimeNano / Math.pow(10.0, 9));
    }

    /**
     * This can be used to update the feedforward equation's input, if applicable.
     *
     * @param input the input into the feedforward equation.
     *
     * @noinspection unused
     */
    public void updateFeedForwardInput(double input) {
        feedForwardInput = input;
    }

    /**
     * This resets all the PIDF's error and position values, as well as the time stamps.
     */
    public void reset() {
        previousError = 0;
        error = 0;
        errorProportional = 0;
        errorDecelerated = 0;
        position = 0;
        targetPosition = 0;
        errorIntegral = 0;
        errorDerivative = 0;
        errorSteadyState = 0;
        previousUpdateTimeNano = System.nanoTime();
        reset = true;
    }

    /**
     * This is used to set the target position if the PIDF is being run with current position and
     * target position inputs rather than error inputs.
     *
     * @param set this sets the target position.
     * @noinspection unused
     */
    public void setTargetPosition(double set) {
        targetPosition = set;
    }

    /**
     * This returns the target position of the PIDF.
     *
     * @return this returns the target position.
     * @noinspection unused
     */
    public double getTargetPosition() {
        return targetPosition;
    }

    /**
     * This is used to set the coefficients of the PIDF.
     *
     * @param set the coefficients that the PIDF will use.
     */
    public void setCoefficients(PIDFCoefficients set) {
        coefficients = set;
    }

    /**
     * This returns the PIDF's current coefficients.
     *
     * @return this returns the current coefficients.
     */
    public PIDFCoefficients getCoefficients() {
        return coefficients;
    }

    /**
     * This sets the proportional (P) coefficient of the PIDF only.
     *
     * @param set this sets the P coefficient.
     * @noinspection unused
     */
    public void setP(double set) {
        coefficients.P = set;
    }

    /**
     * This returns the proportional (P) coefficient of the PIDF.
     *
     * @return this returns the P coefficient.
     */
    public double P() {
        return coefficients.P;
    }

    /**
     * This sets the integral (I) coefficient of the PIDF only.
     *
     * @param set this sets the I coefficient.
     * @noinspection unused
     */
    public void setI(double set) {
        coefficients.I = set;
    }

    /**
     * This returns the integral (I) coefficient of the PIDF.
     *
     * @return this returns the I coefficient.
     */
    public double I() {
        return coefficients.I;
    }

    /**
     * This sets the derivative (D) coefficient of the PIDF only.
     *
     * @param set this sets the D coefficient.
     */
    public void setD(double set) {
        coefficients.D = set;
    }

    /**
     * This returns the derivative (D) coefficient of the PIDF.
     *
     * @return this returns the D coefficient.
     */
    public double D() {
        return coefficients.D;
    }

    /**
     * This sets the feedforward (F) constant of the PIDF only.
     *
     * @param set this sets the F constant.
     */
    public void setF(double set) {
        coefficients.F = set;
    }

    /**
     * This returns the feedforward (F) constant of the PIDF.
     *
     * @return this returns the F constant.
     */
    public double F() {
        return coefficients.getCoefficient(feedForwardInput);
    }

    /**
     * This sets the steady state (S) coefficient of the PIDF only.
     *
     * @param set this sets the S coefficient.
     */
    public void setS(double set) {
        coefficients.S = set;
    }

    /**
     * This returns the steady state (S) coefficient of the PIDF.
     *
     * @return this returns the S coefficient.
     */
    public double S() {
        return coefficients.S;
    }

    /**
     * This returns the current error of the PIDF.
     *
     * @return this returns the error.
     */
    public double getError() {
        return error;
    }

    /**
     * Format the pid parameters into a string
     *
     * @return formatted string
     */
     public String toStringDegrees() {

        if (output == 0) return "";

        double error =  Math.toDegrees(this.error);
        double errorP = Math.toDegrees(this.errorProportional);
        double errorD = Math.toDegrees(this.errorDerivative);
        double errorI = Math.toDegrees(this.errorIntegral);
        double errorS= Math.toDegrees(this.errorSteadyState);

        String s = String.format("pid: %+6.3f(%+7.3f %+7.3f) = ", output, error, errorDecelerated);
        if (P() != 0 && errorProportional != 0)  s+= String.format("P(%+7.3f %7.3f) ", errorP * P(), errorP);
        if (D() != 0 && errorDerivative != 0)    s+= String.format("D(%+6.3f %+6.3f) ", errorD * D(), errorD);
        if (I() != 0 && errorIntegral != 0)      s+= String.format("I(%+6.3f %+6.3f) ", errorI * I(), errorI);
        if (S() != 0 && errorSteadyState != 0)   s+= String.format("S(%+.3f %+.3f) ", errorS * S(), errorS);
        return (s);
    }

    @Override
    @NonNull
    public String toString() {
        if (output == 0) return "";

        String s = String.format("pid: %+6.3f(%+7.3f %+7.3f) = ", output, error, errorDecelerated);
        if (P() != 0 && errorProportional != 0)  s+= String.format("P(%+7.3f %+7.3f) ", errorProportional * P(), errorProportional);
        if (D() != 0 && errorDerivative != 0)    s+= String.format("D(%+6.3f %+6.3f) ", errorDerivative * D(), errorDerivative);
        if (I() != 0 && errorIntegral != 0)      s+= String.format("I(%+6.3f %+6.3f) ", errorIntegral * I(), errorIntegral);
        if (S() != 0 && errorSteadyState != 0)   s+= String.format("S(%+6.3f %+6.3f) ", errorSteadyState * S(), errorSteadyState);
        return (s);
    }
}
