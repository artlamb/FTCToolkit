package utils;

import kotlin.jvm.JvmField;

/**
 * This is the CustomPIDFCoefficients class. This class handles holding coefficients for PIDF
 * controllers.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/5/2024
 */
public class PIDFCoefficients {
    @JvmField public double P;
    @JvmField public double I;
    @JvmField public double D;
    @JvmField public double F;
    @JvmField public double S;
    public double pExponent;
    public double sExponent;
    public double sError;
    public double sThreshold;

    public FeedForwardConstant feedForwardConstantEquation;

    private boolean usingEquation;

    /**
     * This creates a new CustomPIDFCoefficients with constant coefficients.
     *
     * @param p the coefficient for the proportional factor.
     * @param i the coefficient for the integral factor.
     * @param d the coefficient for the derivative factor.
     * @param f the coefficient for the feedforward factor.
     */
    public PIDFCoefficients(double p, double i, double d, double f) {
        P = p;
        I = i;
        D = d;
        F = f;
        this.pExponent = 1;
    }

    /**
     * This creates a new CustomPIDFCoefficients with constant coefficients.
     *
     * @param p the coefficient for the proportional factor.
     * @param pExponent power to raise p to
     * @param s the coefficient for the steady state factor.
     * @param sExponent power to raise s to
     */
    public PIDFCoefficients(double p, double pExponent, double s, double sExponent, double sError, double sThreshold) {
        P = p;
        S = s;
        this.pExponent = pExponent;
        this.sExponent = sExponent;
        this.sError = sError;
        this.sThreshold = sThreshold;
    }

    /**
     * This creates a new CustomPIDFCoefficients with constant PID coefficients and a variable
     * feedforward equation using a FeedForwardConstant.
     *
     * @param p the coefficient for the proportional factor.
     * @param i the coefficient for the integral factor.
     * @param d the coefficient for the derivative factor.
     * @param f the equation for the feedforward factor.
     */
    public PIDFCoefficients(double p, double i, double d, FeedForwardConstant f) {
        usingEquation = true;
        P = p;
        I = i;
        D = d;
        feedForwardConstantEquation = f;
    }

    /**
     * This returns the coefficient for the feedforward factor.
     *
     * @param input this is inputted into the feedforward equation, if applicable. If there's no
     *              equation, then any input can be used.
     * @return This returns the coefficient for the feedforward factor.
     */
    public double getCoefficient(double input) {
        if (!usingEquation) return F;
        return feedForwardConstantEquation.getConstant(input);
    }
}
