package frc.robot.utility.Pathing;

import frc.robot.utility.Setpoint;
import frc.robot.utility.Pathing.profile.AdaptiveSampler;
import frc.robot.utility.Pathing.profile.VelocityProfile;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;


/** A cubic Bezier curve path object.
 * @author fwen2026 */
public class Path {
    private final Setpoint startPose;
    private final Setpoint endPose;

    private final Pose2d startControlHeading;
    private final Pose2d endControlHeading;

    private final TrapezoidProfile.State startRotation;
    private TrapezoidProfile.State currentRotation;
    private final TrapezoidProfile.State endRotation;

    private final TrapezoidProfile rotationProfile;

    // Curve-sampling + velocity planning
    private final AdaptiveSampler sampler;
    private final VelocityProfile velocityProfile;
    private double currentArc;

    private double x3 = 0.0;
    private double x2 = 0.0;
    private double x1 = 0.0;
    private double x0 = 0.0;

    private double y3 = 0.0;
    private double y2 = 0.0;
    private double y1 = 0.0;
    private double y0 = 0.0;


    /** Defines a {@link Path} in 2 dimensions. Translation follows a Bézier curve with a
     * planned velocity profile that respects the tangential-accel and centripetal-accel
     * limits ahead of time. Rotation is handled by a separate trapezoidal profile.
     *
     * @param startPose The start pose
     * @param endPose The end pose
     * @param startControlHeading Control point defining the initial heading
     * @param endControlHeading Control point defining the final heading
     * @param maxVelocity Max path-tangent velocity
     * @param maxAt Max tangential acceleration
     * @param maxOmega Max angular velocity, rad/s
     * @param maxAlpha Max angular acceleration, rad/s^2
     * @param maxCentripetal Max centripetal acceleration; 0 disables the curvature cap
     */
    public Path(Setpoint startPose, Setpoint endPose, Pose2d startControlHeading, Pose2d endControlHeading,
                        double maxVelocity, double maxAt, double maxOmega, double maxAlpha, double maxCentripetal) {
        this.startPose = startPose;
        this.endPose = endPose;
        this.startControlHeading = startControlHeading;
        this.endControlHeading = endControlHeading;

        this.x3 = -startPose.x + 3 * startControlHeading.getX() - 3 * endControlHeading.getX() + endPose.x;
        this.x2 = 3 * startPose.x - 6 * startControlHeading.getX() + 3 * endControlHeading.getX();
        this.x1 = -3 * startPose.x + 3 * startControlHeading.getX();
        this.x0 = startPose.x;

        this.y3 = -startPose.y + 3 * startControlHeading.getY() - 3 * endControlHeading.getY() + endPose.y;
        this.y2 = 3 * startPose.y - 6 * startControlHeading.getY() + 3 * endControlHeading.getY();
        this.y1 = -3 * startPose.y + 3 * startControlHeading.getY();
        this.y0 = startPose.y;

        // Adaptive sampling concentrates samples where the curve bends.
        this.sampler = new AdaptiveSampler(this::calculatePosition, this::getCurvature,
                0.005 /* 5 mm chord tolerance */, 6 /* ~64 min samples */, 12);

        this.velocityProfile = new VelocityProfile(
                sampler.sSamples(),
                sampler.kSamples(),
                maxVelocity,
                maxAt,
                maxCentripetal,
                startPose.getSpeed(),
                endPose.getSpeed());

        this.currentArc = 0.0;

        this.startRotation = new TrapezoidProfile.State(startPose.theta, startPose.omega);
        this.currentRotation = new TrapezoidProfile.State(startPose.theta, startPose.omega);
        this.endRotation = new TrapezoidProfile.State(endPose.theta, endPose.omega);
        this.rotationProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(maxOmega, maxAlpha));
    }

    /** Getter methods*/

    /** Gets start pose */
    public Pose2d getStartPose() {
        return startPose.getSetpointPose();
    }

    /** Gets end pose */
    public Pose2d getEndPose() {
        return endPose.getSetpointPose();
    }

    /** Gets the start pose velocity */
    public double getStartVelocity() {
        return startPose.getSpeed();
    }

    /** Gets the end pose velocity */
    public double getEndVelocity(){
        return endPose.getSpeed();
    }

    /** Gets the start pose heading */
    public Pose2d getStartControlHeading() {
        return startControlHeading;
    }

    /** Gets the end pose heading */
    public Pose2d getEndControlHeading() {
        return endControlHeading;
    }


    /** @return The arc length of the path */
    public double getArcLength(){
        return sampler.getArcLength();
    }

    /** Exposed so {@link PathGroup} can reuse the same planner across stitched segments. */
    public VelocityProfile getVelocityProfile() {
        return velocityProfile;
    }

    /** Exposed so {@link PathGroup} can concatenate per-segment arc-length samples. */
    public double[] getSamplerArcs() {
        return sampler.sSamples();
    }

    /** Exposed so {@link PathGroup} can concatenate per-segment curvature samples. */
    public double[] getSamplerCurvatures() {
        return sampler.kSamples();
    }

    /** Exposed for {@link PathChain}'s pose-based closest-point projection. */
    public AdaptiveSampler getSampler() { return sampler; }


    /** Calculates position of curve with time interval t
     * @param t the time parameter, in the range [0, 1]
     * @return position of robot at time t
    */
    public Translation2d calculatePosition(double t){
        if(t < 0 || t > 1){
            throw new IllegalArgumentException("you're chopped. (t in 0, 1)");
        }
        double t2 = t * t;
        double t3 = t2 * t;
        double x = x3 * t3 + x2 * t2 + x1 * t + x0;
        double y = y3 * t3 + y2 * t2 + y1 * t + y0;

        return new Translation2d(x, y);
    }

    /** Calculates the first derivative at point t
     * @param t the time parameter, in the range [0, 1]
     */
    public Translation2d calculateFirstDerivative(double t){
        if(t < 0 || t > 1){
            throw new IllegalArgumentException("you're chopped. (t in 0, 1)");
        }
        double t2 = t * t;
        double dx = 3 * x3 * t2 + 2 * x2 * t + x1;
        double dy = 3 * y3 * t2 + 2 * y2 * t + y1;

        return new Translation2d(dx, dy);
    }

    /** Calculates the second derivative at point t
     * @param t the time parameter, in the range [0, 1]
     */
    public Translation2d calculateSecondDerivative(double t){
        if(t < 0 || t > 1){
            throw new IllegalArgumentException("you're chopped. (t in 0, 1)");
        }
        double ddx = 6 * x3 * t + 2 * x2;
        double ddy = 6 * y3 * t + 2 * y2;

        return new Translation2d(ddx, ddy);
    }


    /** Inverse of the arc-length parametrization, delegated to the adaptive sampler. */
    public double getTimeForArcLength(double arcLength){
        return sampler.getTimeForArcLength(arcLength);
    }


    /** Calculates the curvature of our path at time t, for better velocity control when turning.
     * @param t the time parameter
     * @return The curvature of the path at time t.
     */
    public double getCurvature(double t){
        if(t < 0 || t > 1){
            throw new IllegalArgumentException("you're chopped. (t in 0, 1)");
        }
        Translation2d firstDerivative = calculateFirstDerivative(t);
        Translation2d secondDerivative = calculateSecondDerivative(t);
        double norm = firstDerivative.getNorm();
        return (firstDerivative.getX() * secondDerivative.getY() - firstDerivative.getY() * secondDerivative.getX())
                / (norm * norm * norm);
    }


    /** This is another helper method to compute the normalized velocity vector T(t).
     * @param t the time parameter
     * @return A normalized vector representing the direction of the path at time t.
     */
    public Translation2d getNormalizedVelocityVector(double t) {
        if(t < 0 || t > 1){
            throw new IllegalArgumentException("you're chopped. (t in 0, 1)");
        }
        Translation2d tangentVector = calculateFirstDerivative(t);
        return tangentVector.div(tangentVector.getNorm() + 0.001);
    }


    /** Pose-based follower. Projects the robot's translation onto the path to find the
     *  current arc length, then commands the point one lookahead step (v·dt) ahead —
     *  so the commanded velocity is nonzero and the robot has somewhere to go. Rotation
     *  tracks a time-based trapezoidal profile interpolated by arc progress; that profile
     *  is read via the robot's reported heading to keep angle-modulus sane.
     *
     *  <p>The projected arc is monotone-clamped: once we've passed a point, we don't go
     *  back. This prevents oscillation if the robot overshoots near the end.
     *
     *  @param robotPose current robot pose, in field coordinates
     *  @param dt        lookahead window (typically the control loop period, 0.02 s)
     */
    public Setpoint calculate(Pose2d robotPose, double dt) {
        // 1. Closest-point projection, clamped to not go backward.
        double arcAtRobot = sampler.closestArc(
                robotPose.getX(), robotPose.getY(), currentArc);
        currentArc = arcAtRobot;

        // 2. Lookahead target: where the robot should be dt seconds from now at the
        //    planned speed at its current arc.
        double vPlanned = velocityProfile.velocityAt(arcAtRobot);
        double targetArc = Math.min(arcAtRobot + vPlanned * dt, getArcLength());
        double commandedVelocity = velocityProfile.velocityAt(targetArc);
        double timeParam = sampler.getTimeForArcLength(targetArc);
        Translation2d translationalTarget = calculatePosition(timeParam);
        Translation2d normalizedVel = getNormalizedVelocityVector(timeParam);

        // 3. Rotation — interpolate goal heading proportional to arc progress at the
        //    robot's current position (not the lookahead) so rotation doesn't race ahead.
        double arcFraction = getArcLength() > 1e-6 ? arcAtRobot / getArcLength() : 1.0;
        double totalRotation = MathUtil.angleModulus(endRotation.position - startRotation.position);
        double intermediateGoal = startRotation.position + arcFraction * totalRotation;

        double rotationalPose = robotPose.getRotation().getRadians();
        double errorToGoal = MathUtil.angleModulus(intermediateGoal - rotationalPose);
        double errorToCurrent = MathUtil.angleModulus(currentRotation.position - rotationalPose);

        State interpGoal = new State(rotationalPose + errorToGoal, 0.0);
        currentRotation.position = rotationalPose + errorToCurrent;

        State rotationalSetpoint = rotationProfile.calculate(dt, currentRotation, interpGoal);
        currentRotation.position = rotationalSetpoint.position;
        currentRotation.velocity = rotationalSetpoint.velocity;

        return new Setpoint(translationalTarget.getX(),
                            translationalTarget.getY(),
                            rotationalSetpoint.position,
                            normalizedVel.getX() * commandedVelocity,
                            normalizedVel.getY() * commandedVelocity,
                            rotationalSetpoint.velocity);
    }

    /** Current projected arc length (updated on each {@link #calculate}). */
    public double getCurrentArc() { return currentArc; }

    /** Reset the monotone arc clamp — call this if the robot is being re-seeded to a
     *  point earlier on the path (e.g. after a pose reset). */
    public void resetProgress() { this.currentArc = 0.0; }
}
