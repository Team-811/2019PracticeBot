/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Utility;

/**
 * Add your docs here.
 */
public class MecanumDrive 
{
    private Output driveOutput;

    public MecanumDrive()
    {
        driveOutput = new Output();
    }

    public Output tankMecanumDrive(double leftValue, double rightValue, double strafeValue, double deadbandValue)
    {
        double topLeftValue = handleDeadband(leftValue - strafeValue, deadbandValue);
        double bottomLeftValue = handleDeadband(leftValue + strafeValue, deadbandValue);
        double topRightValue = handleDeadband(rightValue + strafeValue, deadbandValue);
        double bottomRightValue = handleDeadband(rightValue - strafeValue, deadbandValue);

        driveOutput.updateOutput(topLeftValue, topRightValue, bottomLeftValue, bottomRightValue);

        return driveOutput;
    }

    public Output arcadeMecanumDrive(double forwardValue, double rotationValue, double strafeValue, double deadbandValue, int inverted)
    {
        rotationValue = handleDeadband(rotationValue, deadbandValue);
        double topLeftValue = handleDeadband((forwardValue - rotationValue) + strafeValue, deadbandValue)*inverted;
        double bottomLeftValue = handleDeadband((forwardValue + rotationValue) - strafeValue, deadbandValue)*inverted;
        double topRightValue = handleDeadband((forwardValue - rotationValue) - strafeValue, deadbandValue)*inverted;
        double bottomRightValue = handleDeadband((forwardValue - rotationValue) + strafeValue, deadbandValue)*inverted;

        driveOutput.updateOutput(topLeftValue, topRightValue, bottomLeftValue, bottomRightValue);

        return driveOutput;
    }

    public Output curvatureMecanumDrive(double throttle, double wheel, boolean isQuickTurn, boolean isHighGear, double strafeValue, double deadbandValue)
    {

     /**
     * Helper to implement "Cheesy Drive". "Cheesy Drive" simply means that the "turning" stick controls the curvature
    * of the robot's path rather than its rate of heading change. This helps make the robot more controllable at high
    * speeds. Also handles the robot's quick turn functionality - "quick turn" overrides constant-curvature turning for
    * turn-in-place maneuvers.
     */


    final double kThrottleDeadband = 0.02;
    final double kWheelDeadband = 0.02;

    // These factor determine how fast the wheel traverses the "non linear" sine curve.
    final double kHighWheelNonLinearity = 0.65;
    final double kLowWheelNonLinearity = 0.5;

    final double kHighNegInertiaScalar = 4.0;

    final double kLowNegInertiaThreshold = 0.65;
    final double kLowNegInertiaTurnScalar = 3.5;
    final double kLowNegInertiaCloseScalar = 4.0;
    final double kLowNegInertiaFarScalar = 5.0;

    final double kHighSensitivity = 0.65;
    final double kLowSensitiity = 0.65;

    final double kQuickStopDeadband = 0.5;
    final double kQuickStopWeight = 0.1;
    final double kQuickStopScalar = 5.0;

    double mOldWheel = 0.0;
    double mQuickStopAccumlator = 0.0;
    double mNegInertiaAccumlator = 0.0;

        wheel = handleDeadband(wheel, kWheelDeadband);
        throttle = handleDeadband(throttle, kThrottleDeadband);

        double negInertia = wheel - mOldWheel;
        mOldWheel = wheel;

        double wheelNonLinearity;
        if (isHighGear) {
            wheelNonLinearity = kHighWheelNonLinearity;
            final double denominator = Math.sin(Math.PI / 2.0 * wheelNonLinearity);
            // Apply a sin function that's scaled to make it feel better.
            wheel = Math.sin(Math.PI / 2.0 * wheelNonLinearity * wheel) / denominator;
            wheel = Math.sin(Math.PI / 2.0 * wheelNonLinearity * wheel) / denominator;
        } else {
            wheelNonLinearity = kLowWheelNonLinearity;
            final double denominator = Math.sin(Math.PI / 2.0 * wheelNonLinearity);
            // Apply a sin function that's scaled to make it feel better.
            wheel = Math.sin(Math.PI / 2.0 * wheelNonLinearity * wheel) / denominator;
            wheel = Math.sin(Math.PI / 2.0 * wheelNonLinearity * wheel) / denominator;
            wheel = Math.sin(Math.PI / 2.0 * wheelNonLinearity * wheel) / denominator;
        }

        double leftPwm, rightPwm, overPower;
        double sensitivity;

        double angularPower;
        double linearPower;

        // Negative inertia!
        double negInertiaScalar;
        if (isHighGear) {
            negInertiaScalar = kHighNegInertiaScalar;
            sensitivity = kHighSensitivity;
        } else {
            if (wheel * negInertia > 0) {
                // If we are moving away from 0.0, aka, trying to get more wheel.
                negInertiaScalar = kLowNegInertiaTurnScalar;
            } else {
                // Otherwise, we are attempting to go back to 0.0.
                if (Math.abs(wheel) > kLowNegInertiaThreshold) {
                    negInertiaScalar = kLowNegInertiaFarScalar;
                } else {
                    negInertiaScalar = kLowNegInertiaCloseScalar;
                }
            }
            sensitivity = kLowSensitiity;
        }
        double negInertiaPower = negInertia * negInertiaScalar;
        mNegInertiaAccumlator += negInertiaPower;

        wheel = wheel + mNegInertiaAccumlator;
        if (mNegInertiaAccumlator > 1) {
            mNegInertiaAccumlator -= 1;
        } else if (mNegInertiaAccumlator < -1) {
            mNegInertiaAccumlator += 1;
        } else {
            mNegInertiaAccumlator = 0;
        }
        linearPower = throttle;

        // Quickturn!
        if (isQuickTurn) {
            if (Math.abs(linearPower) < kQuickStopDeadband) {
                double alpha = kQuickStopWeight;
                mQuickStopAccumlator = (1 - alpha) * mQuickStopAccumlator
                        + alpha * limit(wheel, 1.0) * kQuickStopScalar;
            }
            overPower = 1.0;
            angularPower = wheel;
        } else {
            overPower = 0.0;
            angularPower = Math.abs(throttle) * wheel * sensitivity - mQuickStopAccumlator;
            if (mQuickStopAccumlator > 1) {
                mQuickStopAccumlator -= 1;
            } else if (mQuickStopAccumlator < -1) {
                mQuickStopAccumlator += 1;
            } else {
                mQuickStopAccumlator = 0.0;
            }
        }

        rightPwm = leftPwm = linearPower;
        leftPwm += angularPower;
        rightPwm -= angularPower;

        if (leftPwm > 1.0) {
            rightPwm -= overPower * (leftPwm - 1.0);
            leftPwm = 1.0;
        } else if (rightPwm > 1.0) {
            leftPwm -= overPower * (rightPwm - 1.0);
            rightPwm = 1.0;
        } else if (leftPwm < -1.0) {
            rightPwm += overPower * (-1.0 - leftPwm);
            leftPwm = -1.0;
        } else if (rightPwm < -1.0) {
            leftPwm += overPower * (-1.0 - rightPwm);
            rightPwm = -1.0;
        }


        double topLeftValue = handleDeadband(leftPwm + strafeValue, deadbandValue);
        double bottomLeftValue = handleDeadband(leftPwm - strafeValue, deadbandValue);
        double topRightValue = handleDeadband(rightPwm - strafeValue, deadbandValue);
        double bottomRightValue = handleDeadband(rightPwm + strafeValue, deadbandValue); 

        driveOutput.updateOutput(topLeftValue, topRightValue, bottomLeftValue, bottomRightValue);

        return driveOutput;
    }

    private double handleDeadband(double val, double deadband) {
        return (Math.abs(val) > Math.abs(deadband)) ? val : 0.0;
    }

    private double limit(double input, double limit)
    {
        if(input > limit )
        {
            return limit;
        }
        else if(input < -limit)
        {
            return -limit;
        }
        else
        {
            return input;
        }
    }
        
}


    

