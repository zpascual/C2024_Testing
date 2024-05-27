package com.team1678.lib.util;

import edu.wpi.first.math.geometry.Rotation2d;
import org.opencv.core.Mat;

import java.util.List;

public class Util {

    public static final double kEpsilon = 1e-12;

    /**
     * Limits the given input to the given magnitude.
     */
    public static double limit(double v, double maxMagnitude) {
        return limit(v, -maxMagnitude, maxMagnitude);
    }

    public static double limit(double v, double min, double max) {
        return Math.min(max, Math.max(min, v));
    }

    public static boolean epsilonEquals(double a, double b, double epsilon) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }

    public static boolean epsilonEquals(double a, double b) {
        return epsilonEquals(a, b, kEpsilon);
    }

    public static boolean allCloseTo(List<Double> list, double value, double epsilon) {
        boolean result = true;
        for (Double value_in : list) {
            result &= epsilonEquals(value_in, value, epsilon);
        }
        return result;
    }

    public static double normalize(double current, double test){
        if(current > test) return current;
        return test;
    }

    /**
     * Convert from inches to meters
     * @param length Units in inches
     * @return length in meters
     */
    public static double convertInchesToMeters (double length) {
        return length * 0.0254;
    }

    /**
     * Convert from meters to inches
     * @param length Units in meters
     * @return length in inches
     */
    public static double convertMetersToInches (double length) {
        return length / 0.0254;
    }

    public static double boundAngle0To360Scope(double angle) {
        while(angle >= 360.0) {angle -= 360.0;}
        while(angle < 0.0) {angle += 360.0;}
        return angle;
    }

    public static double placeInAppropriate0To360Scope(double scopeReference, double newAngle){
        double lowerBound;
        double upperBound;
        double lowerOffset = scopeReference % 360;
        if(lowerOffset >= 0){
            lowerBound = scopeReference - lowerOffset;
            upperBound = scopeReference + (360 - lowerOffset);
        }else{
            upperBound = scopeReference - lowerOffset;
            lowerBound = scopeReference - (360 + lowerOffset);
        }
        while(newAngle < lowerBound){
            newAngle += 360;
        }
        while(newAngle > upperBound){
            newAngle -= 360;
        }
        if(newAngle - scopeReference > 180){
            newAngle -= 360;
        }else if(newAngle - scopeReference < -180){
            newAngle += 360;
        }
        return newAngle;
    }

    public static boolean shouldReverse(Rotation2d desiredAngle, Rotation2d currentAngle) {
        double angleDifference = Math.abs(desiredAngle.minus(currentAngle).getRadians());
        double oppositeAngleDifference = Math.abs(desiredAngle.minus(currentAngle.rotateBy(Rotation2d.fromDegrees(180.0))).getRadians());
        return oppositeAngleDifference < angleDifference;
    }

    public static double deadBand(double val, double deadband) {
        return (Math.abs(val) > Math.abs(deadband)) ? val : 0.0;
    }

    public static double scaledDeadband(double val, double maxVal, double deadband) {
        double deadbandValue = deadBand(val, deadband);
        return epsilonEquals(deadbandValue, 0.0) ? 0.0 : Math.signum(deadbandValue) * ((Math.abs(deadbandValue) - deadband) / (maxVal - deadband));
    }

    public static boolean isInRange(double val, double range1, double range2) {
        double max, min;
        if (range1 < range2) {
            min = range1;
            max = range2;
        } else {
            min = range2;
            max = range1;
        }
        return min <= val && val <= max;
    }

}
