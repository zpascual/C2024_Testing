package com.team1678.lib.swerve;

import com.team1678.frc2024.Constants;
import com.team1678.frc2024.RobotState;
import com.team1678.lib.control.RadiusController;
import com.team1678.lib.control.SwerveHeadingController;
import com.team254.lib.geometry.Pose2d;

public class TargetLockedFieldRelativeController implements IDriveController {
    public static TargetLockedFieldRelativeController mInstance;
    public SwerveHeadingController mSwerveHeadingController = SwerveHeadingController.getInstance();
    public RadiusController mRadiusController = RadiusController.getInstance();

    private RobotState mRobotState = RobotState.getInstance();

    public static TargetLockedFieldRelativeController getInstance() {
        if (mInstance == null) {
            mInstance = new TargetLockedFieldRelativeController();
        }
        return mInstance;
    }

    @Override
    public ChassisSpeeds transform(DriveInput driveInput, Pose2d robotPose) {
        mRadiusController.setRadiusControllerState(RadiusController.RadiusControllerState.OFF);
        if((mSwerveHeadingController
                .getHeadingControllerState() == SwerveHeadingController.HeadingControllerState.POLAR_SNAP
                && mSwerveHeadingController.isAtGoal()) || driveInput.changeToMaintainTargetHeading()) {
            mSwerveHeadingController.setHeadingControllerState(SwerveHeadingController.HeadingControllerState.POLAR_MAINTAIN);
            mSwerveHeadingController.setGoal(mSwerveHeadingController.calculateAngleToOrigin(mRobotState.getLatestFieldToVehicle()));
        } else {
            mSwerveHeadingController.setHeadingControllerState(SwerveHeadingController.HeadingControllerState.POLAR_SNAP);
            mSwerveHeadingController.setGoal(mSwerveHeadingController.calculateAngleToOrigin(mRobotState.getLatestFieldToVehicle()));
        }
        return ChassisSpeeds.fromFieldRelativeSpeeds(
                driveInput.getThrottle() * Constants.SwerveConfig.kMaxLinearVelocity * Constants.kTranslationScaler,
                driveInput.getStrafe() * Constants.SwerveConfig.kMaxLinearVelocity * Constants.kTranslationScaler,
                mSwerveHeadingController.update(robotPose.getRotation().getDegrees()) * Constants.SwerveConfig.kMaxAngularVelocity,
                robotPose.getRotation());
    }
}
