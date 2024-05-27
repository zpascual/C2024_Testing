package com.team1678.lib.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.littletonrobotics.junction.Logger;

public class LogUtil {

    public static void recordRotation2dDeg(String key, Rotation2d rotation2d) {
        Logger.recordOutput(key, rotation2d.getRotations());
    }

    public static void recordRotation2dRad(String key, Rotation2d rotation2d) {
        Logger.recordOutput(key, rotation2d.getRadians());
    }

}
