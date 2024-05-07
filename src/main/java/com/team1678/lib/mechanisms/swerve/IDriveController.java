package com.team1678.lib.mechanisms.swerve;

import com.team254.lib.geometry.Pose2d;

public interface IDriveController {
    ChassisSpeeds transform(DriveInput driveInput, Pose2d robotPose);
}
