package frc.robot.utility.Pathing;

import frc.robot.utility.Setpoint;
import frc.robot.utility.Pathing.profile.VelocityProfile;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;

import java.util.List;

/**
 * A sequence of {@link Path} segments stitched into a single trajectory with a joint
 * velocity profile. Generalizes {@link PathGroup} to any number of segments — use this
 * whenever A* returns more than one intermediate waypoint.
 *
 * <p>Translation: one {@link VelocityProfile} spans the concatenated arc length, so the
 * backward decel pass sees through every junction and the curvature cap is respected
 * across the whole chain. Rotation: a single trapezoidal profile from the first
 * segment's start heading to the last segment's end heading.
 *
 * <p>C0 continuity at junctions is guaranteed because neighboring segments share the
 * same waypoint as endpoints. C1 (tangent) continuity is the caller's responsibility —
 * {@link PathCalculator#getControlPoints} already places the incoming/outgoing handles
 * colinearly, so chains built through that path are smooth.
 */
public class PathChain {
    private final Path[] segments;
    private final double[] segmentOffsets;   // cumulative arc length at the start of each segment
    private final double totalLength;

    private final VelocityProfile jointProfile;
    private double currentArc;

    private final TrapezoidProfile.State startRotation;
    private final TrapezoidProfile.State currentRotationState;
    private final TrapezoidProfile.State endRotation;
    private final TrapezoidProfile rotationProfile;

    public PathChain(List<Path> segments,
                     double maxVelocity, double maxAt,
                     double maxOmega, double maxAlpha,
                     double maxCentripetal) {
        if (segments == null || segments.isEmpty()) {
            throw new IllegalArgumentException("PathChain requires at least one segment");
        }
        this.segments = segments.toArray(new Path[0]);
        this.segmentOffsets = new double[this.segments.length];
        double acc = 0.0;
        for (int i = 0; i < this.segments.length; i++) {
            segmentOffsets[i] = acc;
            acc += this.segments[i].getArcLength();
        }
        this.totalLength = acc;

        // Concatenate per-segment arc + curvature samples, de-duplicating the shared
        // junction sample between neighbors.
        int totalSamples = 0;
        for (int i = 0; i < this.segments.length; i++) {
            int n = this.segments[i].getSamplerArcs().length;
            totalSamples += (i == 0 ? n : n - 1);
        }
        double[] sJoint = new double[totalSamples];
        double[] kJoint = new double[totalSamples];
        int write = 0;
        for (int i = 0; i < this.segments.length; i++) {
            double[] s = this.segments[i].getSamplerArcs();
            double[] k = this.segments[i].getSamplerCurvatures();
            int start = (i == 0) ? 0 : 1;
            for (int j = start; j < s.length; j++) {
                sJoint[write] = segmentOffsets[i] + s[j];
                kJoint[write] = k[j];
                write++;
            }
        }

        this.jointProfile = new VelocityProfile(
                sJoint, kJoint,
                maxVelocity, maxAt, maxCentripetal,
                this.segments[0].getStartVelocity(),
                this.segments[this.segments.length - 1].getEndVelocity());

        this.currentArc = 0.0;

        double startHeading = this.segments[0].getStartPose().getRotation().getRadians();
        double endHeading = this.segments[this.segments.length - 1].getEndPose().getRotation().getRadians();
        this.startRotation = new TrapezoidProfile.State(startHeading, 0);
        this.currentRotationState = new TrapezoidProfile.State(startHeading, 0);
        this.endRotation = new TrapezoidProfile.State(endHeading, 0);
        this.rotationProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(maxOmega, maxAlpha));
    }

    public int size() { return segments.length; }
    public double getTotalLength() { return totalLength; }
    public Path getSegment(int i) { return segments[i]; }

    /** Pose-based follower over the full chain. Projects the robot onto each segment
     *  (respecting monotone progress across the whole chain), then commands a lookahead
     *  point v·dt ahead on the joint velocity profile. Rotation: time-based trapezoidal
     *  between chain start heading and end heading. */
    public Setpoint calculate(Pose2d robotPose, double dt) {
        // 1. Closest-point projection across all segments. Start searching from whichever
        //    segment contains currentArc — monotonicity means we never revisit earlier ones.
        int startSeg = segments.length - 1;
        for (int i = 0; i < segments.length; i++) {
            if (currentArc <= segmentOffsets[i] + segments[i].getArcLength() - 1e-6) {
                startSeg = i; break;
            }
        }

        double bestArc = currentArc;
        double bestDist2 = Double.POSITIVE_INFINITY;
        for (int i = startSeg; i < segments.length; i++) {
            double localMin = (i == startSeg)
                    ? Math.max(0.0, currentArc - segmentOffsets[i])
                    : 0.0;
            double localArc = segments[i].getSampler().closestArc(
                    robotPose.getX(), robotPose.getY(), localMin);
            double globalArc = segmentOffsets[i] + localArc;
            // Evaluate distance at that projected point.
            double t = segments[i].getTimeForArcLength(localArc);
            Translation2d p = segments[i].calculatePosition(t);
            double dx = p.getX() - robotPose.getX();
            double dy = p.getY() - robotPose.getY();
            double d2 = dx * dx + dy * dy;
            if (d2 < bestDist2) { bestDist2 = d2; bestArc = globalArc; }
        }
        currentArc = bestArc;

        // 2. Lookahead.
        double vPlanned = jointProfile.velocityAt(currentArc);
        double targetArc = Math.min(currentArc + vPlanned * dt, totalLength);
        double commandedVelocity = jointProfile.velocityAt(targetArc);

        // 3. Resolve target segment + local arc.
        int seg = segments.length - 1;
        for (int i = 0; i < segments.length; i++) {
            if (targetArc <= segmentOffsets[i] + segments[i].getArcLength() - 1e-6) {
                seg = i; break;
            }
        }
        double localTarget = Math.min(targetArc - segmentOffsets[seg], segments[seg].getArcLength());
        double timeParam = segments[seg].getTimeForArcLength(localTarget);
        Translation2d target = segments[seg].calculatePosition(timeParam);
        Translation2d tangent = segments[seg].getNormalizedVelocityVector(timeParam);

        // 4. Rotation.
        double currentRotation = robotPose.getRotation().getRadians();
        double errorToGoal = MathUtil.angleModulus(endRotation.position - currentRotation);
        double errorToSetpoint = MathUtil.angleModulus(startRotation.position - currentRotation);
        endRotation.position = currentRotation + errorToGoal;
        currentRotationState.position = currentRotation + errorToSetpoint;

        State rotSetpoint = rotationProfile.calculate(dt, currentRotationState, endRotation);
        currentRotationState.position = rotSetpoint.position;
        currentRotationState.velocity = rotSetpoint.velocity;

        return new Setpoint(
                target.getX(), target.getY(),
                rotSetpoint.position,
                tangent.getX() * commandedVelocity,
                tangent.getY() * commandedVelocity,
                rotSetpoint.velocity);
    }

    public double getCurrentArc() { return currentArc; }
    public void resetProgress() { this.currentArc = 0.0; }
}
