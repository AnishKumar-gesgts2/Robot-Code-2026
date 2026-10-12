package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
    @AutoLog
    public static class IndexerIOInputs {
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
        public double floorRPM = 0.0;
        public double tunnelRPM = 0.0;
    }

    default void updateInputs(IndexerIOInputs inputs) {}

    /** Independent mechanism-side speed targets. Zero stops the respective motor. */
    default void setFloorRPM(double rpm) {}

    default void setTunnelRPM(double rpm) {}

    default void stop() {}
}
