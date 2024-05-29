package com.team1678.lib.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;

import java.util.Optional;

public class Twist2dHelper {
    public static Twist2d mirror(Twist2d other) {
        return new Twist2d(other.dx, -other.dy, -other.dtheta);
    }

    public static Twist2d interpolate(Twist2d current, Twist2d other, double x) {
        return new Twist2d(Util.interpolate(current.dx, other.dx, x),
                Util.interpolate(current.dy, other.dy, x),
                Util.interpolate(current.dtheta, other.dtheta, x));
    }

    public static double calculateNorm(Twist2d twist2d) {
        if (twist2d.dy == 0.0) {
            return Math.abs(twist2d.dx);
        }
        return Math.hypot(twist2d.dx, twist2d.dy);
    }

    public static Optional<Rotation2d> getCourse(Twist2d other) {
        if (Math.pow(other.dx, 2) + Math.pow(other.dy, 2) > Util.kEpsilon) {
            return Optional.of(new Rotation2d(other.dx, other.dy));
        }
        return Optional.empty();
    }
}
