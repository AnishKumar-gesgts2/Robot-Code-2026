package frc.robot.utility.Pathing.profile;

import edu.wpi.first.math.geometry.Translation2d;

import java.util.ArrayList;
import java.util.function.DoubleFunction;
import java.util.function.DoubleUnaryOperator;

/**
 * Builds a parameter-to-arc-length table for a parametric curve by recursive midpoint
 * subdivision. Curves get dense samples where they actually bend and sparse samples on
 * straight runs — no wasted resolution.
 *
 * <p>Decoupled from {@link frc.robot.utility.Pathing.Path} on purpose: accepts plain
 * function refs so it can be unit-tested with any curve.
 */
public class AdaptiveSampler {
    private final double[] ts;
    private final double[] ss;
    private final double[] ks;
    private final double[] xs;      // cached sample positions for closest-point queries
    private final double[] ys;
    private final double totalLength;

    /**
     * @param position        t in [0,1] -> point on the curve
     * @param curvature       t in [0,1] -> signed curvature
     * @param chordTolerance  meters; midpoint deviation below this stops subdivision
     * @param minDepth        forces at least 2^minDepth segments even on straight sections
     * @param maxDepth        cap to keep recursion bounded (typical 12 -> up to 4096 samples)
     */
    public AdaptiveSampler(DoubleFunction<Translation2d> position,
                           DoubleUnaryOperator curvature,
                           double chordTolerance,
                           int minDepth,
                           int maxDepth) {
        ArrayList<Double> tList = new ArrayList<>(256);
        tList.add(0.0);
        subdivide(position, tList, 0.0, 1.0, chordTolerance, 0, minDepth, maxDepth);

        int n = tList.size();
        this.ts = new double[n];
        this.ss = new double[n];
        this.ks = new double[n];
        this.xs = new double[n];
        this.ys = new double[n];

        Translation2d prev = position.apply(tList.get(0));
        ts[0] = tList.get(0);
        ss[0] = 0.0;
        ks[0] = curvature.applyAsDouble(tList.get(0));
        xs[0] = prev.getX();
        ys[0] = prev.getY();
        for (int i = 1; i < n; i++) {
            double t = tList.get(i);
            Translation2d cur = position.apply(t);
            ts[i] = t;
            ss[i] = ss[i - 1] + cur.getDistance(prev);
            ks[i] = curvature.applyAsDouble(t);
            xs[i] = cur.getX();
            ys[i] = cur.getY();
            prev = cur;
        }
        this.totalLength = ss[n - 1];
    }

    public double[] xSamples() { return xs; }
    public double[] ySamples() { return ys; }

    /** Finds the arc length of the sample point closest to {@code (x, y)}, restricted to
     *  sample indices in {@code [minArc, totalLength]}. Linear scan — O(n). For our
     *  sample counts (~200) this is ~2 μs on a roboRIO and avoids the subtlety of
     *  analytical Newton refinement converging to a local minimum. */
    public double closestArc(double x, double y, double minArc) {
        int startIdx = 0;
        if (minArc > 0) {
            startIdx = Math.max(0, Math.min(ss.length - 1,
                    java.util.Arrays.binarySearch(ss, minArc) < 0
                            ? -java.util.Arrays.binarySearch(ss, minArc) - 1
                            : java.util.Arrays.binarySearch(ss, minArc)));
        }
        double bestDist2 = Double.POSITIVE_INFINITY;
        int bestIdx = startIdx;
        for (int i = startIdx; i < xs.length; i++) {
            double dx = xs[i] - x, dy = ys[i] - y;
            double d2 = dx * dx + dy * dy;
            if (d2 < bestDist2) { bestDist2 = d2; bestIdx = i; }
        }
        // Refine between bestIdx's neighbors by projecting onto the adjacent segments.
        double refined = ss[bestIdx];
        if (bestIdx > 0) {
            double cand = projectOntoSegment(bestIdx - 1, bestIdx, x, y);
            if (cand >= minArc && cand >= 0 && distanceAtArc2(cand, x, y) < bestDist2) {
                refined = cand;
                bestDist2 = distanceAtArc2(cand, x, y);
            }
        }
        if (bestIdx + 1 < xs.length) {
            double cand = projectOntoSegment(bestIdx, bestIdx + 1, x, y);
            if (cand >= minArc && distanceAtArc2(cand, x, y) < bestDist2) {
                refined = cand;
            }
        }
        return Math.max(minArc, Math.min(totalLength, refined));
    }

    private double projectOntoSegment(int a, int b, double x, double y) {
        double ax = xs[a], ay = ys[a];
        double bx = xs[b], by = ys[b];
        double dx = bx - ax, dy = by - ay;
        double lenSq = dx * dx + dy * dy;
        if (lenSq < 1e-12) return ss[a];
        double t = ((x - ax) * dx + (y - ay) * dy) / lenSq;
        t = Math.max(0, Math.min(1, t));
        return ss[a] + t * (ss[b] - ss[a]);
    }

    private double distanceAtArc2(double s, double x, double y) {
        int idx = searchArc(s);
        double denom = ss[idx + 1] - ss[idx];
        double frac = denom > 1e-12 ? (s - ss[idx]) / denom : 0.0;
        double px = xs[idx] + frac * (xs[idx + 1] - xs[idx]);
        double py = ys[idx] + frac * (ys[idx + 1] - ys[idx]);
        double dx = px - x, dy = py - y;
        return dx * dx + dy * dy;
    }

    private void subdivide(DoubleFunction<Translation2d> pos, ArrayList<Double> out,
                           double a, double b, double tol,
                           int depth, int minDepth, int maxDepth) {
        double m = 0.5 * (a + b);
        if (depth >= maxDepth) {
            out.add(b);
            return;
        }
        if (depth >= minDepth) {
            Translation2d pa = pos.apply(a);
            Translation2d pb = pos.apply(b);
            Translation2d pm = pos.apply(m);
            double err = pm.getDistance(pa.plus(pb).div(2.0));
            if (err < tol) {
                out.add(b);
                return;
            }
        }
        subdivide(pos, out, a, m, tol, depth + 1, minDepth, maxDepth);
        subdivide(pos, out, m, b, tol, depth + 1, minDepth, maxDepth);
    }

    public double getArcLength() {
        return totalLength;
    }

    public int size() {
        return ts.length;
    }

    public double[] tSamples() {
        return ts;
    }

    public double[] sSamples() {
        return ss;
    }

    public double[] kSamples() {
        return ks;
    }

    public double getTimeForArcLength(double s) {
        s = clamp(s, 0.0, totalLength);
        int idx = searchArc(s);
        double denom = ss[idx + 1] - ss[idx];
        double frac = denom > 1e-12 ? (s - ss[idx]) / denom : 0.0;
        return ts[idx] + frac * (ts[idx + 1] - ts[idx]);
    }

    public double getCurvatureAtArcLength(double s) {
        s = clamp(s, 0.0, totalLength);
        int idx = searchArc(s);
        double denom = ss[idx + 1] - ss[idx];
        double frac = denom > 1e-12 ? (s - ss[idx]) / denom : 0.0;
        return ks[idx] + frac * (ks[idx + 1] - ks[idx]);
    }

    private int searchArc(double s) {
        int lo = 0, hi = ss.length - 1;
        while (lo + 1 < hi) {
            int mid = (lo + hi) >>> 1;
            if (ss[mid] <= s) lo = mid;
            else hi = mid;
        }
        return lo;
    }

    private static double clamp(double v, double lo, double hi) {
        return v < lo ? lo : (v > hi ? hi : v);
    }
}
