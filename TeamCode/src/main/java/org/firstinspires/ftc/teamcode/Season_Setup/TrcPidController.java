/*
 * Copyright (c) 2015 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.Season_Setup;

import java.util.EmptyStackException;
import java.util.Locale;
import java.util.Stack;

/**
 * This class implements a PID controller. A PID controller takes a target set point and an input from a feedback
 * device to calculate the output power of an effector usually a motor or a set of motors.
 */
public class TrcPidController
{
    protected static final String moduleName = "TrcPidController";
    protected static final boolean debugEnabled = false;
    protected static final boolean tracingEnabled = false;
    protected static final boolean useGlobalTracer = false;

    /**
     * This class encapsulates all the PID coefficients into a single object and makes it more efficient to pass them
     * around.
     */
    public static class PidCoefficients
    {
        public double kP;
        public double kI;
        public double kD;
        public double kF;
        public double iZone;

        /**
         * Constructor: Create an instance of the object.
         *
         * @param kP specifies the Proportional constant.
         * @param kI specifies the Integral constant.
         * @param kD specifies the Differential constant.
         * @param kF specifies the Feed forward constant.
         * @param iZone specifies the integral zone.
         */
        public PidCoefficients(double kP, double kI, double kD, double kF, double iZone)
        {
            this.kP = Math.abs(kP);
            this.kI = Math.abs(kI);
            this.kD = Math.abs(kD);
            this.kF = Math.abs(kF);
            this.iZone = Math.abs(iZone);
        }   //PidCoefficients

        /**
         * Constructor: Create an instance of the object.
         *
         * @param kP specifies the Proportional constant.
         * @param kI specifies the Integral constant.
         * @param kD specifies the Differential constant.
         * @param kF specifies the Feed forward constant.
         */
        public PidCoefficients(double kP, double kI, double kD, double kF)
        {
            this(kP, kI, kD, kF, 0.0);
        }   //PidCoefficients

        /**
         * Constructor: Create an instance of the object.
         */
        public PidCoefficients()
        {
            this(1.0, 0.0, 0.0, 0.0);
        }   //PidCoefficients

        /**
         * Constructor: Create an instance of the object.
         *
         * @param kP specifies the Proportional constant.
         * @param kI specifies the Integral constant.
         * @param kD specifies the Differential constant.
         */
        public PidCoefficients(double kP, double kI, double kD)
        {
            this(kP, kI, kD, 0.0);
        }   //PidCoefficients

        /**
         * Constructor: Create an instance of the object.
         *
         * @param kP specifies the Proportional constant.
         */
        public PidCoefficients(double kP)
        {
            this(kP, 0.0, 0.0, 0.0);
        }   //PidCoefficients

        /**
         * This method returns all PID coefficients in string form.
         *
         * @return PID coefficients string.
         */
        @Override
        public String toString()
        {
            return String.format("(%f,%f,%f,%f)", kP, kI, kD, kF);
        }   //toString

        /**
         * This method returns a copy of this object.
         *
         * @return a copy of this object.
         */
        public PidCoefficients clone()
        {
            return new PidCoefficients(kP, kI, kD, kF);
        }   //clone

    }   //class PidCoefficients

    /**
     * PID controller needs input from a feedback device for calculating the output power. Whoever is providing this
     * input must implement this interface.
     */
    public interface PidInput
    {
        /**
         * This method is called by the PID controller to get input data from the feedback device. The feedback
         * device can be motor encoders, gyro, ultrasonic sensor, light sensor etc.
         *
         * @return input value of the feedback device.
         */
        double get();

    }   //interface PidInput

    public static final double DEF_SETTLING_TIME = 0.2;

    private String instanceName;
    private PidCoefficients pidCoefficients;
    private double tolerance;
    private double settlingTime;
    private PidInput pidInput;

    private boolean inverted = false;
    private boolean absSetPoint = false;
    private boolean noOscillation = false;
    private double minTarget = 0.0;
    private double maxTarget = 0.0;
    private double minOutput = -1.0;
    private double maxOutput = 1.0;
    private double outputLimit = 1.0;
    private Double rampRate = null;
    private Stack<Double> outputLimitStack = new Stack<>();

    private double prevTime = 0.0;
    private double currError = 0.0;
    private double totalError = 0.0;
    private double settlingStartTime = 0.0;
    private double setPoint = 0.0;
    private double setPointSign = 1.0;
    private double currInput = 0.0;
    private double output = 0.0;
    private double prevOutputTime = 0.0; // time that getOutput() was called last. Used for ramp rates.

    private double pTerm;
    private double iTerm;
    private double dTerm;
    private double fTerm;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName    specifies the instance name.
     * @param pidCoefficients specifies the PID constants.
     * @param tolerance       specifies the target tolerance.
     * @param settlingTime    specifies the minimum on target settling time.
     * @param pidInput        specifies the input provider.
     */
    public TrcPidController(final String instanceName, PidCoefficients pidCoefficients, double tolerance,
        double settlingTime, PidInput pidInput)
    {
        this.instanceName = instanceName;
        this.pidCoefficients = pidCoefficients;
        this.tolerance = Math.abs(tolerance);
        this.settlingTime = Math.abs(settlingTime);
        this.pidInput = pidInput;
    }   //TrcPidController

    /**
     * Constructor: Create an instance of the object. This constructor is not public. It is only for classes
     * extending this class (e.g. Cascade PID Controller) that cannot make itself as an input provider in its
     * constructor (Java won't allow it). Instead, we provide another protected method setPidInput so it can
     * set the PidInput outside of the super() call.
     *
     * @param instanceName    specifies the instance name.
     * @param pidCoefficients specifies the PID constants.
     * @param tolerance       specifies the target tolerance.
     * @param pidInput        specifies the input provider.
     */
    public TrcPidController(
            final String instanceName, PidCoefficients pidCoefficients, double tolerance, PidInput pidInput)
    {
        this(instanceName, pidCoefficients, tolerance, DEF_SETTLING_TIME, pidInput);
    }   //TrcPidController

    /**
     * This method returns the instance name.
     *
     * @return instance name.
     */
    @Override
    public String toString()
    {
        return instanceName;
    }   //toString

    /**
     * This method sets the set point mode to be absolute. PID controller always calculates the output with an
     * absolute set point comparing to a sensor value representing an absolute input. But by default, it will
     * treat the set point as a value relative to its current input. So it will add the relative set point value
     * to the current input as the absolute set point in its calculation. This method allows the caller to treat
     * the set point as absolute set point.
     *
     * @param absolute specifies true if set point is absolute, false otherwise.
     */
    public synchronized void setAbsoluteSetPoint(boolean absolute)
    {
        final String funcName = "setAbsoluteSetPoint";

        this.absSetPoint = absolute;
    }   //setAbsoluteSetPoint

    /**
     * This method returns true if setpoints are absolute, false otherwise.
     *
     * @return true if setpoints are absolute, false otherwise.
     */
    public synchronized boolean hasAbsoluteSetPoint()
    {
        final String funcName = "hasAbsoluteSetPoint";

        return absSetPoint;
    }   //hasAbsoluteSetPoint

    /**
     * This method enables/disables NoOscillation mode. In PID control, if the PID constants are not tuned quite
     * correctly, it may cause oscillation that could waste a lot of time. In some scenarios, passing the target
     * beyond the tolerance may be acceptable. This method allows the PID controller to declare "On Target" even
     * though it passes the target beyond tolerance so it doesn't oscillate.
     *
     * @param noOscillation specifies true to enable no oscillation, false to disable.
     */
    public synchronized void setNoOscillation(boolean noOscillation)
    {
        final String funcName = "setNoOscillation";

        this.noOscillation = noOscillation;
    }   //setNoOscillation

    /**
     * This method returns the current PID coefficients.
     *
     * @return current PID coefficients.
     */
    public synchronized PidCoefficients getPidCoefficients()
    {
        final String funcName = "getPidCoefficients";

        return pidCoefficients;
    }   //getPidCoefficients

    /**
     * This method sets new PID coefficients.
     *
     * @param pidCoefficients specifies new PID coefficients.
     */
    public synchronized void setPidCoefficients(PidCoefficients pidCoefficients)
    {
        final String funcName = "setPidCoefficients";

        this.pidCoefficients = pidCoefficients;
    }   //setPidCoefficients

    /**
     * This method sets the ramp rate of the PID controller output. It is sometimes useful to limit the acceleration
     * of the output of the PID controller. For example, the strafing PID controller on a mecanum drive base may
     * benefit from a lower acceleration to minimize wheel slipperage.
     *
     * @param rampRate specifies the ramp rate in percent power per second.
     */
    public synchronized void setRampRate(Double rampRate)
    {
        this.rampRate = rampRate;
    }   //setRampRate

    /**
     * This method sets a new target tolerance.
     *
     * @param tolerance specifies the new target tolerance.
     */
    public synchronized void setTargetTolerance(double tolerance)
    {
        this.tolerance = tolerance;
    }   //setTargetTolerance

    /**
     * This method sets a range limit on the target set point.
     *
     * @param minTarget specifies the target set point lower range limit.
     * @param maxTarget specifies the target set point higher range limit.
     */
    public synchronized void setTargetRange(double minTarget, double maxTarget)
    {
        final String funcName = "setTargetRange";

        this.minTarget = minTarget;
        this.maxTarget = maxTarget;
    }   //setTargetRange

    /**
     * This method sets a range limit on the calculated output. It is very useful to limit the output range to
     * less than full power for scenarios such as using mecanum wheels on a drive train to prevent wheel slipping
     * or slow down a PID drive in order to detect a line etc.
     *
     * @param minOutput specifies the PID output lower range limit.
     * @param maxOutput specifies the PID output higher range limit.
     */
    public synchronized void setOutputRange(double minOutput, double maxOutput)
    {
        final String funcName = "setOutputRange";

        if (maxOutput <= minOutput)
        {
            throw new IllegalArgumentException("maxOutput must be greater than minOutput");
        }

        if (Math.abs(minOutput) == Math.abs(maxOutput))
        {
            outputLimit = maxOutput;
        }

        this.minOutput = minOutput;
        this.maxOutput = maxOutput;
    }   //setOutputRange

    /**
     * This method sets the output to the range -limit to +limit. It calls setOutputRange. If the caller wants
     * to limit the output power symmetrically, this is the method to call, not setOutputRange.
     *
     * @param limit specifies the output limit as a positive number.
     */
    public synchronized void setOutputLimit(double limit)
    {
        limit = Math.abs(limit);
        setOutputRange(-limit, limit);
    }   //setOutputLimit

    /**
     * This method returns the last set output limit. It is sometimes useful to temporarily change the output
     * range of the PID controller for an operation and restore it afterwards. This method allows the caller to
     * save the last set output limit and restore it later on.
     *
     * @return last set output limit.
     */
    public synchronized double getOutputLimit()
    {
        final String funcName = "getOutputLimit";

        return outputLimit;
    }   //getOutputLimit

    /**
     * This method saves the current output limit of the PID controller and sets it to the given new limit.
     * This is useful if the caller wants to temporarily set a limit for an operation and restore it afterwards.
     * Note: this is implemented with a stack so it is assuming the saving and restoring calls are nested in
     * nature. If this is called in a multi-threading environment where saving and restoring can be interleaved
     * by different threads, unexpected result may happen. It is recommended to avoid this type of scenario if
     * possible.
     *
     * @param limit specifies the new output limit.
     * @return return the previous output limit.
     */
    public synchronized double saveAndSetOutputLimit(double limit)
    {
        final String funcName = "saveAndSetOutputLimit";
        double prevLimit = outputLimit;

        outputLimitStack.push(outputLimit);
        setOutputLimit(limit);

        return prevLimit;
    }   //saveAndSetOutputLimit

    /**
     * This method restores the last saved output limit and return its value. If there was no previous call to
     * saveAndSetOutputLimit, the current output limit is returned and the limit is not changed.
     *
     * @return last saved output limit.
     */
    public synchronized double restoreOutputLimit()
    {
        final String funcName = "restoreOutputLimit";
        double limit;

        try
        {
            limit = outputLimitStack.pop();
            setOutputLimit(limit);
        }
        catch (EmptyStackException e)
        {
            //
            // There was no previous saveAndSetOutputLimit call, don't do anything and just return the current
            // output limit.
            //
            limit = outputLimit;
        }

        return limit;
    }   //restoreOutputLimit

    /**
     * This method returns the current set point value.
     *
     * @return current set point.
     */
    public synchronized double getTarget()
    {
        final String funcName = "getTarget";

        return setPoint;
    }   //getTarget

    /**
     * This methods sets the target set point.
     *
     * @param target    specifies the target set point.
     * @param warpSpace specifies the warp space object if the target is in one, null if not.
     */
    public void setTarget(double target, TrcWarpSpace warpSpace)
    {
        final String funcName = "setTarget";

        //
        // Read from input device without holding a lock on this object, since this could
        // be a long-running call.
        //
        final double input = pidInput.get();

        synchronized (this)
        {
            if (!absSetPoint)
            {
                //
                // Set point is relative, add target to current input to get absolute set point.
                //
                setPoint = input + target;
                currError = target;
            }
            else
            {
                //
                // Set point is absolute, use as is but optimize it if it is in warp space.
                //
                setPoint = target;
                if (warpSpace != null)
                {
                    setPoint = warpSpace.getOptimizedTarget(setPoint, input);
                }
                currError = setPoint - input;
            }

            if (inverted)
            {
                currError = -currError;
            }

            setPointSign = Math.signum(currError);
            //
            // If there is a valid target range, limit the set point to this range.
            //
            if (maxTarget > minTarget)
            {
                if (setPoint > maxTarget)
                {
                    setPoint = maxTarget;
                }
                else if (setPoint < minTarget)
                {
                    setPoint = minTarget;
                }
            }

            totalError = 0.0;
            prevTime = settlingStartTime = TrcUtil.getCurrentTime();
            //
            // Only init the prevOutputTime if this setTarget is called after a reset()
            // If it's called mid-operation, we don't want to reset the prevOutputTime clock
            //
            if (prevOutputTime == 0.0)
            {
                prevOutputTime = prevTime;
            }
        }

    }   //setTarget

    /**
     * This methods sets the target set point.
     *
     * @param target specifies the target set point.
     */
    public void setTarget(double target)
    {
        setTarget(target, null);
    }   //setTarget

    /**
     * This method returns the error of a previous output calculation.
     *
     * @return previous error.
     */
    public synchronized double getError()
    {
        final String funcName = "getError";

        return currError;
    }   //getError

    /**
     * This method resets the PID controller clearing the set point, error, total error and output.
     */
    public synchronized void reset()
    {
        final String funcName = "reset";

        currError = 0.0;
        prevTime = 0.0;
        prevOutputTime = 0.0;
        totalError = 0.0;
        setPoint = 0.0;
        setPointSign = 1.0;
        output = 0.0;
    }   //reset

    /**
     * This method determines if we have reached the set point target. It is considered on target if the previous
     * error is smaller than the tolerance and is maintained for at least settling time. If NoOscillation mode is
     * set, it is considered on target if we are within tolerance or pass target regardless of setting time.
     *
     * @return true if we reached target, false otherwise.
     */
    public synchronized boolean isOnTarget()
    {
        final String funcName = "isOnTarget";

        boolean onTarget = false;

        if (noOscillation)
        {
            //
            // Don't allow oscillation, so if we are within tolerance or we pass target, just quit.
            // If setPointSign is positive, it means the target is "forward". So if currError <= tolerance,
            // it means we are either within tolerance or have passed the target.
            // If setPointSign is negative, it means the target is "backward". So if -currError <= tolerance,
            // it means we are either within tolerance or have passed the target.
            //
            if (currError * setPointSign <= tolerance)
            {
                onTarget = true;
            }
        }
        else if (Math.abs(currError) > tolerance)
        {
            settlingStartTime = TrcUtil.getCurrentTime();
        }
        else if (TrcUtil.getCurrentTime() >= settlingStartTime + settlingTime)
        {
            onTarget = true;
        }

        return onTarget;
    }   //isOnTarget

    /**
     * This method returns the current PID input value.
     *
     * @return current PID input value.
     */
    public double getCurrentInput()
    {
        return pidInput.get();
    }   //getCurrentInput

    /**
     * This method calculates the PID output applying the PID equation to the given set point target and current
     * input value.
     *
     * @return PID output value.
     */
    public double getOutput()
    {
        //
        // Read from input device without holding a lock on this object, since this could
        // be a long-running call.
        //
        final double currentInputValue = pidInput.get();

        synchronized (this)
        {
            final String funcName = "getOutput";
            double prevError = currError;
            double currTime = TrcUtil.getCurrentTime();
            double deltaTime = currTime - prevTime;

            prevTime = currTime;
            currInput = currentInputValue;
            currError = setPoint - currInput;
            if (inverted)
            {
                currError = -currError;
            }

            if (pidCoefficients.kI != 0.0)
            {
                //
                // Make sure the total error doesn't get wound up too much exceeding maxOutput.
                //
                double potentialGain = (totalError + currError * deltaTime) * pidCoefficients.kI;
                if (potentialGain >= maxOutput)
                {
                    totalError = maxOutput / pidCoefficients.kI;
                }
                else if (potentialGain > minOutput)
                {
                    totalError += currError * deltaTime;
                }
                else
                {
                    totalError = minOutput / pidCoefficients.kI;
                }
            }

            pTerm = pidCoefficients.kP * currError;
            iTerm = pidCoefficients.kI * totalError;
            dTerm = deltaTime > 0.0 ? pidCoefficients.kD * (currError - prevError) / deltaTime : 0.0;
            fTerm = pidCoefficients.kF * setPoint;
            double lastOutput = output;
            output = pTerm + iTerm + dTerm + fTerm;

            output = TrcUtil.clipRange(output, minOutput, maxOutput);

            if (rampRate != null)
            {
                if (prevOutputTime != 0.0)
                {
                    double dt = currTime - prevOutputTime;
                    double maxChange = rampRate * dt;
                    double change = output - lastOutput;
                    change = TrcUtil.clipRange(change, -maxChange, maxChange);
                    output = lastOutput + change;
                }
                prevOutputTime = currTime;
            }

            return output;
        }
    }   //getOutput

}   //class TrcPidController