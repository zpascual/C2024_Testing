package com.team1678.lib.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;

public class Pose2dHelper {
    public static Pose2d mirror(Pose2d other) {
        return new Pose2d(new Translation2d(other.getX(), -other.getY()), Rotation2dHelper.inverse(other.getRotation()));
    }

    public static Pose2d inverse(Pose2d current, Pose2d other) {
        Rotation2d invertedRotation = Rotation2dHelper.inverse(current.getRotation());
        return new Pose2d(current.getTranslation().unaryMinus().rotateBy(invertedRotation), invertedRotation);
    }

    public static Twist2d log(Pose2d other) {
        final double dTheta = other.getRotation().getRadians();
        final double half_dTheta = 0.5 * dTheta;
        final double cosMinusOne = other.getRotation().getCos() - 1.0;
        double halfThetaByTanOfHalfTheta;
        if (Math.abs(cosMinusOne) < Util.kEpsilon) {
            halfThetaByTanOfHalfTheta = 1.0 - 1.0 / 12.0 * Math.pow(dTheta, 2);
        } else {
            halfThetaByTanOfHalfTheta = -(half_dTheta * other.getRotation().getSin()) / cosMinusOne;
        }
        final Translation2d translation2d = other.getTranslation().rotateBy(new Rotation2d(halfThetaByTanOfHalfTheta, -half_dTheta));
        return new Twist2d(translation2d.getX(), translation2d.getY(), dTheta);
    }

    public static double distance(Pose2d current, Pose2d other) {
        return Twist2dHelper.calculateNorm(current.log(inverse(current, other).relativeTo(other)));
    }

    public static Pose2d inverse(Pose2d other) {
        Rotation2d invertedRotation = Rotation2dHelper.inverse(other.getRotation());
        return new Pose2d(other.getTranslation().unaryMinus().rotateBy(invertedRotation), invertedRotation);
    }
}
