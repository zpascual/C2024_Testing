package com.team254.lib.trajectory.timing;

import com.team1678.lib.swerve.ChassisSpeeds;
import com.team1678.lib.swerve.SwerveDriveKinematics;
import com.team1678.lib.swerve.SwerveKinematicLimits;
import com.team1678.lib.swerve.SwerveSetpointGenerator;
import com.team254.lib.geometry.ICourse2d;
import com.team254.lib.geometry.ICurvature;
import com.team254.lib.geometry.IPose2d;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.IHeadingRate;

public class SwerveDriveDynamicsConstraint<S extends IPose2d<S> & ICourse2d<S> & ICurvature<S> & IHeadingRate<S>> implements TimingConstraint<S> {

    protected final SwerveDriveKinematics kinematics_;
    protected final SwerveKinematicLimits limits_;

    protected final SwerveSetpointGenerator setpoint_generator_;

    public SwerveDriveDynamicsConstraint(final SwerveDriveKinematics kinematics, final SwerveKinematicLimits limits) {
        kinematics_ = kinematics;
        limits_ = limits;
        setpoint_generator_ = new SwerveSetpointGenerator(kinematics);
    }

    @Override
    public double getMaxVelocity(S state) {
        // First check instantaneous velocity and compute a limit based on drive velocity.
        var course = state.getCourse();
        Rotation2d course_local = state.getPose().getRotation().inverse().rotateBy(course.orElse(Rotation2d.kIdentity));
        double vx = course_local.cos();
        double vy = course_local.sin();
        double vtheta = state.getHeadingRate();
        double curvature = state.getCurvature();
        ChassisSpeeds chassis_speeds = new ChassisSpeeds(vx, vy, vtheta);

        var module_states = kinematics_.toSwerveModuleStates(chassis_speeds);
        double max_vel = Double.POSITIVE_INFINITY;
        for (var module : module_states) {
            max_vel = Math.min(max_vel, limits_.kMaxDriveVelocity / Math.abs(module.speedMetersPerSecond));
        }

        return max_vel;
    }

    @Override
    public MinMaxAcceleration getMinMaxAcceleration(S state,
                                                    double velocity) {
        // Just check drive acceleration limits.
        return new MinMaxAcceleration(-limits_.kMaxDriveAcceleration, limits_.kMaxDriveAcceleration);
    }
}