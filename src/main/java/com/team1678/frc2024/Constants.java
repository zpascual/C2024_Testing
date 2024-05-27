package com.team1678.frc2024;

import com.team1678.lib.motion.MotionProfileConstraints;
import com.team1678.lib.util.Util;
import org.opencv.core.Mat;

public class Constants {
    public static final double kLooperDt =  0.01;

    public static final double kEpsilon = 1E-3;

    // Config checking
    public static final double kRecheckDeviceIntervalSeconds = 10.0;

    // Driver Control Scaling
    public static final double kTranslationScaler = 0.5;
    public static final double kRotationScaler = 0.2;

    public static class CANTiming {
        public static final int kCANTimeoutMs = 10; // use for important on the fly updates
        public static final int kLongCANTimeoutMs = 100; // use for constructors
        public static final double kCancoderBootAllowanceSeconds = 10.0;
    }

    public static class SwerveConfig {
        public static final double kWheelDiameter = 4.0; // inches
        public static final double kTrackWidth = 24.0; // inches
        public static final double kWheelBase = 24.0; // inches
        public static final double kMaxVoltage = 12.0;
        public static final double kMaxLinearVelocity = 5.05; // m/s
        public static final double kMaxLinearAccel = 4.4; // m/s^2
        public static final double kMaxAngularVelocity = kMaxLinearVelocity / Math.hypot(kTrackWidth / 2.0, kWheelBase / 2.0);
        public static final double kMaxAngularAccel = kMaxLinearAccel / Math.hypot(kTrackWidth / 2.0, kWheelBase / 2.0);
        public static final boolean kUseVelocityDrive = true;
        public static final double kAzmithRatio = 15.0;
        public static final double kDriveRatio = 5.8;
        public static final double kAzmithTolerance = 10.0;
        public static final double kMotionMagicDriveToleranceInches = 2.0;
        public static final double kDriveRotorRevPerMeter = kDriveRatio / (Math.PI * Util.convertInchesToMeters(kWheelDiameter));
    }

    public static class SwerveHeadingController {
        public static final double kSwerveHeadingControllerErrorTolerance = 1.5; // degree error

        public static final double kSnapSwerveHeadingKp = 0.05;
        public static final double kSnapSwerveHeadingKi = 0.0;
        public static final double kSnapSwerveHeadingKd = 0.0075;

        public static final double kMaintainSwerveHeadingKpHighVelocity = 0.0225;
        public static final double kMaintainSwerveHeadingKiHighVelocity = 0.0;
        public static final double kMaintainSwerveHeadingKdHighVelocity = 0.003;

        public static final double kMaintainSwerveHeadingKpLowVelocity = 0.02;  // 0.01;
        public static final double kMaintainSwerveHeadingKiLowVelocity = 0.0;
        public static final double kMaintainSwerveHeadingKdLowVelocity = 0.0;

        // Swerve heading controller gains
        public static final double kHeadingControllerKp = 2.54;
        public static final double kHeadingControllerKi = 0.0;
        public static final double kHeadingControllerKd = 0.0;
        public static final double kHeadingControllerKffv = 1.0;
        public static final double kHeadingControllerKffa = 0.0;
        public static final double kHeadingControllerKs = 0.0;

        public static final double kSnapRadiusKp = 2.0;
        public static final double kSnapRadiusKi = 0.0;
        public static final double kSnapRadiusKd = 0.0;

        public static final double kMaintainRadiusKp = 1.5;
        public static final double kMaintainRadiusKi = 0.0;
        public static final double kMaintainRadiusKd = 0.0;
    }

    public static class SwerveMotionProfile {
        public static final MotionProfileConstraints kPosition = new MotionProfileConstraints(
                0.8 * SwerveConfig.kMaxLinearVelocity,
                0.8 * -SwerveConfig.kMaxLinearVelocity,
                0.6 * SwerveConfig.kMaxLinearAccel
        );
        public static final MotionProfileConstraints kHeading = new MotionProfileConstraints(
                0.5 * SwerveConfig.kMaxAngularVelocity,
                0.5 * -SwerveConfig.kMaxAngularVelocity,
                SwerveConfig.kMaxAngularAccel
        );
    }

    public static class PurePursuit {
        public static final double kPathLookaheadTime = 0.25; //seconds
        public static final double kPathMinLookaheadDistance = 12.0; // From 1323 (2019)
        public static final double kAdaptivePathMinLookaheadDistance = 6.0;
        public static final double kAdaptivePathMaxLookaheadDistance = 24.0;
        public static final double kAdaptiveErrorLookaheadCoefficient = 4.0;
    }

    public static class MotorData {
        public static final double kMaxFalconRotationsPerSecond = 6380.0 / 60.0;
        public static final double kMaxFalconEncoderSpeed = 6380.0 * 2048.0 / 600.0;
        public static final double kFalconMotionMagicFeedForward = 1023.0 / kMaxFalconEncoderSpeed;

        public static final double kMaxKrakenRotationsPerSecond = 6000.0 / 60.0;
        public static final double kMaxKrakenEncoderSpeed = 6000.0 * 2048.0 / 600.0;
        public static final double kKrakenMotionMagicFeedForward = 12.0 / kMaxKrakenRotationsPerSecond;

        public static final double kTalonFxEncoderResolution = 2048.0;
    }

}
