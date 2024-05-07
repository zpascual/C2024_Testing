package com.team254.lib.trajectory;

import com.team1678.frc2024.Constants;
import com.team1678.frc2024.planners.DriveMotionPlanner;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Pose2dWithMotion;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Translation2d;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.timing.TimedState;
import com.team254.lib.trajectory.timing.TimingConstraint;
import com.team254.lib.trajectory.timing.VelocityLimitRegionConstraint;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.List;

public class TrajectoryGenerator {
    private static final double kMaxAccel = Constants.SwerveConfig.kMaxLinearAccel * 0.8;
    private static final double kMaxVoltage = 9.0;

    private final DriveMotionPlanner mMotionPlanner;
    private TrajectorySet mTrajectorySet = null;

    public TrajectoryGenerator(DriveMotionPlanner motion_planner) {
        mMotionPlanner = motion_planner;
    }

    public void generateTrajectories() {
        if (mTrajectorySet == null) {
            System.out.println("Generating trajectories...");
            mTrajectorySet = new TrajectorySet();
            System.out.println("Finished trajectory generation");
        }
    }

    public void forceRegenerateTrajectories() {
        System.out.println("Force Generating trajectories...");
        mTrajectorySet = new TrajectorySet();
        System.out.println("Finished trajectory generation");
    }

    public TrajectorySet getTrajectorySet() {
        return mTrajectorySet;
    }

    public Trajectory<TimedState<Pose2dWithMotion>> generateTrajectory(
            boolean reversed,
            final List<Pose2d> waypoints,
            final List<Rotation2d> headings,
            final List<TimingConstraint<Pose2dWithMotion>> constraints,
            double max_vel,  // m/s
            double max_accel,  // m/s^2
            double max_voltage) {
        return mMotionPlanner.generateTrajectory(reversed, waypoints, headings, constraints, max_vel, max_accel, max_voltage);
    }

    public Trajectory<TimedState<Pose2dWithMotion>> generateTrajectory(
            boolean reversed,
            final List<Pose2d> waypoints,
            final List<Rotation2d> headings,
            final List<TimingConstraint<Pose2dWithMotion>> constraints,
            double start_vel,  // m/s
            double end_vel,  // m/s
            double max_vel,  // m/s
            double max_accel,  // m/s^2
            double max_voltage) {
        return mMotionPlanner.generateTrajectory(reversed, waypoints, headings, constraints, start_vel, end_vel, max_vel, max_accel, max_voltage);
    }

    public class TrajectorySet {
        public final Trajectory<TimedState<Pose2dWithMotion>> testTrajectory;
        public final Trajectory<TimedState<Pose2dWithMotion>> testTrajectory2;


        private List<Trajectory<TimedState<Pose2dWithMotion>>> allTrajectories;

        public List<Trajectory<TimedState<Pose2dWithMotion>>> getAllTrajectories() {
            return allTrajectories;
        }

        private TrajectorySet() {
            allTrajectories = new ArrayList<>();
            testTrajectory = getTestTrajectory();
            allTrajectories.add(testTrajectory);
            testTrajectory2 = getTestTrajectory2();
            allTrajectories.add(testTrajectory2);
        }

        private void convertToM(List<Pose2d> waypoints, List<Rotation2d> headings) {
            for (int i = 0; i < waypoints.size(); ++i) {
                System.out.println("waypoints.add(new Pose2d(" + Units.inchesToMeters(waypoints.get(i).getTranslation().x())
                        + ", " + Units.inchesToMeters(waypoints.get(i).getTranslation().y())
                        + ", Rotation2d.fromDegrees(" + waypoints.get(i).getRotation().getDegrees()
                        + ")));");
                System.out.println("headings.add(Rotation2d.fromDegrees("
                + headings.get(i).getDegrees()+"));");
            }
            System.out.println("\n\n");

        }

        private Trajectory<TimedState<Pose2dWithMotion>> getTestTrajectory() {
            List<Pose2d> waypoints = new ArrayList<>();
            List<Rotation2d> headings = new ArrayList<>();
            waypoints.add(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0)));
            headings.add(Rotation2d.fromDegrees(180.0));
            waypoints.add(new Pose2d(1.0, 0.0, Rotation2d.fromDegrees(0)));
            headings.add(Rotation2d.fromDegrees(180.0));
            waypoints.add(new Pose2d(2.5, 0.0, Rotation2d.fromDegrees(0)));
            headings.add(Rotation2d.fromDegrees(0));
            return generate(waypoints, headings, List.of(), false, 0.8, 1.0);
        }

        private Trajectory<TimedState<Pose2dWithMotion>> getTestTrajectory2() {
            List<Pose2d> waypoints = new ArrayList<>();
            List<Rotation2d> headings = new ArrayList<>();
            waypoints.add(new Pose2d(2.5, 0.0, Rotation2d.fromDegrees(180)));
            headings.add(Rotation2d.fromDegrees(0));
            waypoints.add(new Pose2d(1.0, 0.0, Rotation2d.fromDegrees(180)));
            headings.add(Rotation2d.fromDegrees(180.0));
            waypoints.add(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(180)));
            headings.add(Rotation2d.fromDegrees(180.0));
            return generate(waypoints, headings, List.of(), false, 0.3, 1.0);
        }

        private Trajectory<TimedState<Pose2dWithMotion>> generate(List<Pose2d> waypoints, List<Rotation2d> headings, List<TimingConstraint<Pose2dWithMotion>> constraints, boolean reversed, double percentSpeed, double percentAccel) {
            handleAllianceFlip(waypoints, headings);

            return generateTrajectory(reversed, waypoints, headings, constraints,
                    percentSpeed * Constants.SwerveConfig.kMaxLinearVelocity, percentAccel * kMaxAccel, kMaxVoltage);
        }


        private void handleAllianceFlip(List<Pose2d> waypoints, List<Rotation2d> headings) {
            SmartDashboard.putString("Alliance", DriverStation.getAlliance().toString());
            if (DriverStation.getAlliance().toString().equals(DriverStation.Alliance.Blue.toString())) {
                waypoints.replaceAll(pose2d -> new Pose2d(new Translation2d(pose2d.getTranslation().x(), -pose2d.getTranslation().y()), pose2d.getRotation().inverse()));
                headings.replaceAll(Rotation2d::inverse);
            }
        }
    }
}