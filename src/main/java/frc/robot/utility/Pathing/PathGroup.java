package frc.robot.utility.Pathing;

import frc.robot.utility.Setpoint;
import frc.robot.utility.Pathing.profile.VelocityProfile;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;


public class PathGroup{
    public Path firstPath;
    public Path secondPath;

    public TrapezoidProfile.State startRotation;
    public TrapezoidProfile.State currentRotationState;
    public TrapezoidProfile.State endRotation;

    public TrapezoidProfile rotationProfile;

    // Joint planned velocity profile over the concatenated arc length. This ensures the
    // curvature cap is respected across both segments and that the backward decel pass
    // can "see through" the junction.
    private final VelocityProfile jointProfile;
    private double currentArc;

    /** Creates a PathGroup object. Joint velocity profile enforces curvature limits across both segments. */
    public PathGroup(Path firstPath, Path secondPath, double maxVelocity, double maxAt, double maxOmega, double maxAlpha) {
        this(firstPath, secondPath, maxVelocity, maxAt, maxOmega, maxAlpha, 0.0);
    }

    public PathGroup(Path firstPath, Path secondPath,
                     double maxVelocity, double maxAt, double maxOmega, double maxAlpha,
                     double maxCentripetal) {
        this.firstPath = firstPath;
        this.secondPath = secondPath;

        // Rebuild the joint v(s) from each segment's samples. We re-sample curvature off
        // the segments' own samplers so we don't drop resolution through concatenation.
        double[] s1 = firstPath.getVelocityProfile() != null
                ? firstPath.getSamplerArcs() : new double[]{0, firstPath.getArcLength()};
        double[] k1 = firstPath.getSamplerCurvatures();
        double[] s2 = secondPath.getSamplerArcs();
        double[] k2 = secondPath.getSamplerCurvatures();

        int n1 = s1.length;
        int n2 = s2.length;
        double offset = firstPath.getArcLength();
        // Avoid duplicating the junction sample.
        int total = n1 + n2 - 1;
        double[] sJoint = new double[total];
        double[] kJoint = new double[total];
        for (int i = 0; i < n1; i++) {
            sJoint[i] = s1[i];
            kJoint[i] = k1[i];
        }
        for (int i = 1; i < n2; i++) {
            sJoint[n1 - 1 + i] = offset + s2[i];
            kJoint[n1 - 1 + i] = k2[i];
        }

        this.jointProfile = new VelocityProfile(
                sJoint, kJoint, maxVelocity, maxAt, maxCentripetal,
                firstPath.getStartVelocity(), secondPath.getEndVelocity());
        this.currentArc = 0.0;

        this.startRotation = new TrapezoidProfile.State(firstPath.getStartPose().getRotation().getRadians(), 0);
        this.currentRotationState = new TrapezoidProfile.State(firstPath.getStartPose().getRotation().getRadians(), 0);
        this.endRotation = new TrapezoidProfile.State(secondPath.getEndPose().getRotation().getRadians(), 0);

        this.rotationProfile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(maxOmega, maxAlpha)
        );
    }


    /** Pose-based follower across the two-segment chain. See {@link PathChain#calculate}
     *  for the full scheme — this is the same algorithm with exactly two segments. */
    public Setpoint calculate(Pose2d robotPose, double dt) {
        double totalLength = firstPath.getArcLength() + secondPath.getArcLength();

        // Closest-point projection across both segments, monotone-clamped.
        double bestArc = currentArc;
        double bestDist2 = Double.POSITIVE_INFINITY;

        // First segment (only if we haven't passed it yet).
        if (currentArc < firstPath.getArcLength() - 1e-6) {
            double localMin = Math.max(0.0, currentArc);
            double localArc = firstPath.getSampler().closestArc(
                    robotPose.getX(), robotPose.getY(), localMin);
            double t = firstPath.getTimeForArcLength(localArc);
            Translation2d p = firstPath.calculatePosition(t);
            double dx = p.getX() - robotPose.getX();
            double dy = p.getY() - robotPose.getY();
            double d2 = dx * dx + dy * dy;
            if (d2 < bestDist2) { bestDist2 = d2; bestArc = localArc; }
        }

        // Second segment.
        {
            double localMin = Math.max(0.0, currentArc - firstPath.getArcLength());
            double localArc = secondPath.getSampler().closestArc(
                    robotPose.getX(), robotPose.getY(), localMin);
            double t = secondPath.getTimeForArcLength(localArc);
            Translation2d p = secondPath.calculatePosition(t);
            double dx = p.getX() - robotPose.getX();
            double dy = p.getY() - robotPose.getY();
            double d2 = dx * dx + dy * dy;
            if (d2 < bestDist2) { bestDist2 = d2; bestArc = firstPath.getArcLength() + localArc; }
        }
        currentArc = bestArc;

        // Lookahead on joint profile.
        double vPlanned = jointProfile.velocityAt(currentArc);
        double targetArc = Math.min(currentArc + vPlanned * dt, totalLength);
        double commandedVelocity = jointProfile.velocityAt(targetArc);

        Translation2d normalizedVelocity;
        double timeParam;
        Translation2d translationalTarget;
        if (targetArc <= firstPath.getArcLength() - 1e-6) {
            timeParam = firstPath.getTimeForArcLength(targetArc);
            normalizedVelocity = firstPath.getNormalizedVelocityVector(timeParam);
            translationalTarget = firstPath.calculatePosition(timeParam);
        } else {
            double localArc = Math.min(targetArc - firstPath.getArcLength(), secondPath.getArcLength());
            timeParam = secondPath.getTimeForArcLength(localArc);
            normalizedVelocity = secondPath.getNormalizedVelocityVector(timeParam);
            translationalTarget = secondPath.calculatePosition(timeParam);
        }

        // Rotation.
        double rotationalPose = robotPose.getRotation().getRadians();
        double errorToGoal = MathUtil.angleModulus(endRotation.position - rotationalPose);
        double errorToSetpoint = MathUtil.angleModulus(startRotation.position - rotationalPose);

        endRotation.position = rotationalPose + errorToGoal;
        currentRotationState.position = rotationalPose + errorToSetpoint;

        State rotationalSetpoint = rotationProfile.calculate(dt, currentRotationState, endRotation);
        currentRotationState.position = rotationalSetpoint.position;
        currentRotationState.velocity = rotationalSetpoint.velocity;

        return new Setpoint(translationalTarget.getX(),
                            translationalTarget.getY(),
                            rotationalSetpoint.position,
                            normalizedVelocity.getX() * commandedVelocity,
                            normalizedVelocity.getY() * commandedVelocity,
                            rotationalSetpoint.velocity);
    }

    public double getCurrentArc() { return currentArc; }
    public void resetProgress() { this.currentArc = 0.0; }
}
