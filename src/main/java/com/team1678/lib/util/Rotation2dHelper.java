package com.team1678.lib.util;

import edu.wpi.first.math.geometry.Rotation2d;

public class Rotation2dHelper {
    public static Rotation2d nearestPole(Rotation2d rotation2d) {
        double pole_sin, pole_cos;
        if (Math.abs(rotation2d.getCos()) > Math.abs(rotation2d.getSin())) {
            pole_cos = Math.signum(rotation2d.getCos());
            pole_sin = 0.0;
        } else {
            pole_cos = 0.0;
            pole_sin = Math.signum(rotation2d.getSin());
        }
        return new Rotation2d(pole_cos, pole_sin);
    }

    public static Rotation2d inverse(Rotation2d rotation2d) {
        return new Rotation2d(rotation2d.getCos(), -rotation2d.getSin());
    }
}
