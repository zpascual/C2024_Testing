package com.team1678.lib.util;

import edu.wpi.first.math.geometry.Translation2d;

public class Translation2dHelper {
    public static Translation2d scale(Translation2d translation2d, double scaleFactor) {
        return new Translation2d(translation2d.getX() * scaleFactor, translation2d.getY() * scaleFactor);
    }

    public static double getNorm2(Translation2d other) {
        return Math.pow(other.getX(), 2) + Math.pow(other.getY(), 2);
    }
}
