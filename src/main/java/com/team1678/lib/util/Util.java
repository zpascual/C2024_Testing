package com.team1678.lib.util;

public class Util {

    /**
     * Convert from inches to meters
     * @param length Units in inches
     * @return length in meters
     */
    public double convertToMeters (double length) {
        return length * 0.0254;
    }

    /**
     * Convert from meters to inches
     * @param length Units in meters
     * @return length in inches
     */
    public double convertToInches (double length) {
        return length / 0.0254;
    }

}
