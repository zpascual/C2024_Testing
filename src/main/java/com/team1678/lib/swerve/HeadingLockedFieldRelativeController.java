package com.team1678.lib.swerve;

import com.team1678.frc2024.Constants;
import com.team1678.lib.control.RadiusController;
import com.team1678.lib.control.SwerveHeadingController;
import com.team254.lib.geometry.Pose2d;

public class HeadingLockedFieldRelativeController implements IDriveController {
    public static HeadingLockedFieldRelativeController mInstance;
    public SwerveHeadingController mSwerveHeadingController = SwerveHeadingController.getInstance();
    public RadiusController mRadiusController = RadiusController.getInstance();

    public static HeadingLockedFieldRelativeController getInstance() {
        if (mInstance == null) {
            mInstance = new HeadingLockedFieldRelativeController();
        }
        return mInstance;
    }

    @Override
    public ChassisSpeeds transform(DriveInput driveInput, Pose2d robotPose) {
        mRadiusController.setRadiusControllerState(RadiusController.RadiusControllerState.OFF);
        mSwerveHeadingController.setHeadingControllerState(SwerveHeadingController.HeadingControllerState.SNAP);
        mSwerveHeadingController.setGoal(driveInput.getDesiredCardinalHeading());
        return ChassisSpeeds.fromFieldRelativeSpeeds(
                driveInput.getThrottle() * Constants.SwerveConfig.kMaxLinearVelocity * Constants.kTranslationScaler,
                driveInput.getStrafe() * Constants.SwerveConfig.kMaxLinearVelocity * Constants.kTranslationScaler,
                mSwerveHeadingController.update(robotPose.getRotation().getDegrees()) * Constants.SwerveConfig.kMaxAngularVelocity,
                robotPose.getRotation());
    }
}
