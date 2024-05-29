package com.team1678.lib.geometry;

import com.team1678.lib.util.Pose2dHelper;
import com.team1678.lib.util.Twist2dHelper;
import com.team1678.lib.util.Util;
import com.team254.lib.geometry.State;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;

import java.text.DecimalFormat;
import java.util.Optional;

public class Pose2dWithMotion implements State<Pose2dWithMotion> {
    protected final Pose2d pose;
    protected final Twist2d motionDirection;
    protected final double curvature;
    protected final double dCurvature_dS;
    protected static final Pose2dWithMotion kIdentity = new Pose2dWithMotion();

    public static Pose2dWithMotion identity() {return kIdentity;}

    public Pose2dWithMotion() {
        this.pose = new Pose2d();
        this.motionDirection = new Twist2d(0.0, 0.0, 0.0);
        this.curvature = 0.0;
        this.dCurvature_dS = 0.0;
    }

    public Pose2dWithMotion(final Pose2d pose, double curvature) {
        this.pose = pose;
        this.motionDirection = new Twist2d(0.0, 0.0, 0.0);
        this.curvature = curvature;
        this.dCurvature_dS = 0.0;
    }

    public Pose2dWithMotion(final Pose2d pose, double curvature, double dCurvature_dS) {
        this.pose = pose;
        this.motionDirection = new Twist2d(0.0, 0.0, 0.0);
        this.curvature = curvature;
        this.dCurvature_dS = dCurvature_dS;
    }

    public Pose2dWithMotion(final Pose2d pose, final Twist2d motionDirection, double curvature, double dCurvature_dS) {
        this.pose = pose;
        this.motionDirection = motionDirection;
        this.curvature = curvature;
        this.dCurvature_dS = dCurvature_dS;
    }

    public Pose2dWithMotion(final Translation2d translation2d, final Rotation2d rotation2d, double curvature) {
        this.pose = new Pose2d(translation2d, rotation2d);
        this.motionDirection = new Twist2d(0.0, 0.0, 0.0);
        this.curvature = curvature;
        this.dCurvature_dS = 0.0;
    }

    public Pose2dWithMotion(final Translation2d translation2d, final Rotation2d rotation2d, double curvature, double dCurvature_dS) {
        this.pose = new Pose2d(translation2d, rotation2d);
        this.motionDirection = new Twist2d(0.0, 0.0, 0.0);
        this.curvature = curvature;
        this.dCurvature_dS = dCurvature_dS;
    }

    public Pose2dWithMotion(final Translation2d translation2d, final Rotation2d rotation2d, Twist2d twist2d, double curvature, double dCurvature_dS) {
        this.pose = new Pose2d(translation2d, rotation2d);
        this.motionDirection = twist2d;
        this.curvature = curvature;
        this.dCurvature_dS = dCurvature_dS;
    }

    public final Pose2d getPose() {return this.pose;}
    public final Twist2d getMotionDirection() {return this.motionDirection;}
    public final double getCurvature() {return this.curvature;}
    public final double getDCurvatureDs() {return this.dCurvature_dS;}
    public final Translation2d getTranslation() {return getPose().getTranslation();}
    public final Rotation2d getRotation() {return getPose().getRotation();}

    public Pose2dWithMotion transformBy(Pose2d other) {
        return new Pose2dWithMotion(getPose().relativeTo(other), motionDirection, getCurvature(), getDCurvatureDs());
    }

    public Pose2dWithMotion mirror() {
        return new Pose2dWithMotion(Pose2dHelper.mirror(getPose()), Twist2dHelper.mirror(motionDirection), -getCurvature(), -getDCurvatureDs());
    }

    public Pose2dWithMotion interpolate(final Pose2dWithMotion other, double x) {
        return new Pose2dWithMotion(getPose().interpolate(other.getPose(), x),
                Twist2dHelper.interpolate(this.motionDirection, other.motionDirection, x),
                Util.interpolate(getCurvature(), other.getCurvature(), x),
                Util.interpolate(getDCurvatureDs(), other.getDCurvatureDs(), x));
    }

    public double distance(final Pose2dWithMotion other) {
        Pose2d inversePose = Pose2dHelper.inverse(getPose(), other.getPose()).relativeTo(other.getPose());
        return Twist2dHelper.calculateNorm(getPose().log(inversePose));
    }

    @Override
    public Pose2dWithMotion add(Pose2dWithMotion other) {
        return this.transformBy(other.getPose());
    }

    public boolean equals(final Object other) {
        if (other instanceof Pose2dWithMotion p2dwm) {
            return getPose().equals(p2dwm.getPose()) &&
                    motionDirection.equals(p2dwm.getMotionDirection()) &&
                    Util.epsilonEquals(getCurvature(), p2dwm.getCurvature()) &&
                    Util.epsilonEquals(getDCurvatureDs(), p2dwm.getDCurvatureDs());
        }
        return false;
    }

    public String toCSV() {
        final DecimalFormat fmt = new DecimalFormat("#0.000");
        return String.join(",",
                getPose().toString(),
                fmt.format(motionDirection.dx),
                fmt.format(motionDirection.dy),
                fmt.format(motionDirection.dtheta),
                fmt.format(getCurvature()),
                fmt.format(getDCurvatureDs())
        );
    }

    /**
     * Motion direction is always relative to pose, so it gets rotated for free
     */
    public Pose2dWithMotion rotateBy(Rotation2d other) {
        return new Pose2dWithMotion(getPose().rotateBy(other), getCurvature(), getDCurvatureDs());
    }

    public Optional<Rotation2d> getCourse() {return Twist2dHelper.getCourse(motionDirection);}
    public double getHeadingRate() {return motionDirection.dtheta;}
}
