package com.team1678.frc2024;

import com.ctre.phoenix6.signals.InvertedValue;
import com.team1678.frc2024.subsystems.swerve.SwerveModule.*;
import com.team1678.lib.motion.MotionProfileConstraints;
import com.team1678.lib.util.Util;
import edu.wpi.first.math.geometry.Translation2d;

import java.util.Arrays;
import java.util.List;

public class Constants {
    public static final double kMainThreadDt = 0.02;
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
        public static final double kTrackWidth = 24.0; // inches | side to side
        public static final double kWheelBase = 24.0; // inches | front to back
        public static final double kMaxVoltage = 12.0;
        public static final double kMaxLinearVelocity = 5.05; // m/s
        public static final double kMaxLinearAccel = 4.4; // m/s^2
        public static final double kMaxAngularVelocity = kMaxLinearVelocity / Math.hypot(kTrackWidth / 2.0, kWheelBase / 2.0);
        public static final double kMaxAngularAccel = kMaxLinearAccel / Math.hypot(kTrackWidth / 2.0, kWheelBase / 2.0);
        public static final double kAzmithRatio = 15.0;
        public static final double kDriveRatio = 5.8;
        public static final double kAzmithTolerance = 10.0;
        public static final double kMotionMagicDriveToleranceInches = 2.0;
        public static final double kDriveRotorRevPerMeter = kDriveRatio / (Math.PI * Util.convertInchesToMeters(kWheelDiameter));
    }

    public static class SwerveModules {
        public static final class frontRight {
            public static final double kAngleOffset = Settings.kIsUsingCompBot ? 0.0 : 0.0;
            public static final SwerveMotorInfo kDriveMotorInfo = new SwerveMotorInfo(Ports.FRONT_RIGHT_DRIVE, InvertedValue.Clockwise_Positive);
            public static final SwerveMotorInfo kAzmithMotorInfo = new SwerveMotorInfo(Ports.FRONT_RIGHT_ROTATION, InvertedValue.Clockwise_Positive);
            public static final Translation2d kVehicleToModule = new Translation2d(Util.convertInchesToMeters(SwerveConfig.kTrackWidth/ 2.0), Util.convertInchesToMeters(SwerveConfig.kWheelBase / 2.0));
        }
        public static final class frontLeft {
            public static final double kAngleOffset = Settings.kIsUsingCompBot ? 0.0 : 0.0;
            public static final SwerveMotorInfo kDriveMotorInfo = new SwerveMotorInfo(Ports.FRONT_LEFT_DRIVE, InvertedValue.Clockwise_Positive);
            public static final SwerveMotorInfo kAzmithMotorInfo = new SwerveMotorInfo(Ports.FRONT_LEFT_ROTATION, InvertedValue.Clockwise_Positive);
            public static final Translation2d kVehicleToModule = new Translation2d(-Util.convertInchesToMeters(SwerveConfig.kTrackWidth/ 2.0), Util.convertInchesToMeters(SwerveConfig.kWheelBase / 2.0));

        }
        public static final class rearLeft {
            public static final double kAngleOffset = Settings.kIsUsingCompBot ? 0.0 : 0.0;
            public static final SwerveMotorInfo kDriveMotorInfo = new SwerveMotorInfo(Ports.REAR_LEFT_DRIVE, InvertedValue.Clockwise_Positive);
            public static final SwerveMotorInfo kAzmithMotorInfo = new SwerveMotorInfo(Ports.REAR_LEFT_ROTATION, InvertedValue.Clockwise_Positive);
            public static final Translation2d kVehicleToModule = new Translation2d(-Util.convertInchesToMeters(SwerveConfig.kTrackWidth/ 2.0), -Util.convertInchesToMeters(SwerveConfig.kWheelBase / 2.0));
        }
        public static final class rearRight {
            public static final double kAngleOffset = Settings.kIsUsingCompBot ? 0.0 : 0.0;
            public static final SwerveMotorInfo kDriveMotorInfo = new SwerveMotorInfo(Ports.REAR_RIGHT_DRIVE, InvertedValue.Clockwise_Positive);
            public static final SwerveMotorInfo kAzmithMotorInfo = new SwerveMotorInfo(Ports.REAR_RIGHT_ROTATION, InvertedValue.Clockwise_Positive);
            public static final Translation2d kVehicleToModule = new Translation2d(Util.convertInchesToMeters(SwerveConfig.kTrackWidth/ 2.0), -Util.convertInchesToMeters(SwerveConfig.kWheelBase / 2.0));
        }

        public static final List<Translation2d> kModulePositions = Arrays.asList(frontRight.kVehicleToModule, frontLeft.kVehicleToModule, rearLeft.kVehicleToModule, rearRight.kVehicleToModule);
    }

    public static class SwerveHeadingController {
        public static final double kErrorTolerance = 1.5; // degree error

        public static final double kSnapKp = 0.05;
        public static final double kSnapKi = 0.0;
        public static final double kSnapKd = 0.0075;

        public static final double kMaintainKpHighVelocity = 0.0225;
        public static final double kMaintainKiHighVelocity = 0.0;
        public static final double kMaintainKdHighVelocity = 0.003;

        public static final double kPolarMaintainKp = 0.0;
        public static final double kPolarMaintainKi = 0.0;
        public static final double kPolarMaintainKd = 0.0;

        public static final double kSnapStationaryKp = 0.05;
        public static final double kSnapStationaryKi = 0.0;
        public static final double kSnapStationaryKd = 0.0075;
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
