package com.team1678.lib.mechanisms.swerve;

import com.team1678.frc2024.Constants;
import com.team1678.frc2024.RobotState;
import com.team1678.lib.control.RadiusController;
import com.team1678.lib.control.SwerveHeadingController;
import com.team254.lib.geometry.Pose2d;

public class FieldRelativeController implements IDriveController {
    public static FieldRelativeController mInstance;
    public SwerveHeadingController mSwerveHeadingController = SwerveHeadingController.getInstance();
    public RadiusController mRadiusController = RadiusController.getInstance();

    private RobotState mRobotState = RobotState.getInstance();

    public static FieldRelativeController getInstance() {
        if (mInstance == null) {
            mInstance = new FieldRelativeController();
        }
        return mInstance;
    }

    @Override
    public ChassisSpeeds transform(DriveInput driveInput, Pose2d robotPose) {
        mRadiusController.setRadiusControllerState(RadiusController.RadiusControllerState.OFF);
        if ((mSwerveHeadingController
                .getHeadingControllerState() == SwerveHeadingController.HeadingControllerState.SNAP
                && mSwerveHeadingController.isAtGoal()) || driveInput.changeHeadingSetpoint()) {
            mSwerveHeadingController
                    .setHeadingControllerState(SwerveHeadingController.HeadingControllerState.MAINTAIN);
            mSwerveHeadingController.setGoal(mRobotState.getLatestFieldToVehicle().getRotation().getDegrees());
            return ChassisSpeeds.fromFieldRelativeSpeeds(
                    driveInput.getThrottle() * Constants.SwerveConfig.kMaxLinearVelocity * Constants.kTranslationScaler,
                    driveInput.getStrafe() * Constants.SwerveConfig.kMaxLinearVelocity * Constants.kTranslationScaler,
                    mSwerveHeadingController.update(robotPose.getRotation().getDegrees()) * Constants.SwerveConfig.kMaxAngularAccel,
                    robotPose.getRotation());
        }
        return ChassisSpeeds.fromFieldRelativeSpeeds(
                driveInput.getThrottle() * Constants.SwerveConfig.kMaxLinearVelocity * Constants.kTranslationScaler,
                driveInput.getStrafe() * Constants.SwerveConfig.kMaxLinearVelocity * Constants.kTranslationScaler,
                driveInput.getRotation() * Constants.SwerveConfig.kMaxAngularVelocity * Constants.kRotationScaler,
                robotPose.getRotation());
    }
}
