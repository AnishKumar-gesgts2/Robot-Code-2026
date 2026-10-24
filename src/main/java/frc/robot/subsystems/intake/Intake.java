package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

/** Rack-only hardware test: extend once on teleop enable, then retract and hold. */
public class Intake extends SubsystemBase {
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    private boolean wasTeleopEnabled;
    private boolean pitCoastMode;
    private double enableTime;
    private double target = IntakeConstants.RETRACTED_POSITION_ROTATIONS;

    public enum State {
        RETRACTED_STOPPED, EXTENDED_STOPPED, EXTENDED_SPINNING, EXTENDED_OUTTAKING
    }

    public Intake(IntakeIO io) { this.io = io; }

    // Existing auto/shot callers cannot override this rack-only hardware test.
    public void activate() {}
    public void deactivate() {}
    public void activateOuttake() {}
    public void setExtendedSpinning() {}
    public void setExtendedStopped() {}
    public void setRetracted() {}
    public void startShootingOscillation() {}
    public void stopShootingOscillation() {}
    public boolean isActive() { return false; }
    public boolean isShootingOscillationActive() { return false; }

    public State getState() {
        return target == IntakeConstants.EXTENDED_POSITION_ROTATIONS
            ? State.EXTENDED_STOPPED : State.RETRACTED_STOPPED;
    }

    public boolean isReady() {
        return inputs.configured && inputs.deployConnected && inputs.deployHomed;
    }

    public boolean atDeployTarget() {
        return isReady() && Math.abs(inputs.deployPositionRotations - target)
            <= IntakeConstants.DEPLOY_POSITION_TOLERANCE_ROTATIONS;
    }

    public void setPitCoastMode(boolean enabled) {
        pitCoastMode = enabled;
        io.stop();
        io.setCoastMode(enabled);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
        boolean teleopEnabled = DriverStation.isTeleopEnabled();
        if (teleopEnabled && !wasTeleopEnabled) {
            enableTime = Timer.getFPGATimestamp();
        }
        wasTeleopEnabled = teleopEnabled;
        // Three seconds includes outward travel; this runs once per teleop enable.
        target = teleopEnabled && Timer.getFPGATimestamp() - enableTime
                < IntakeConstants.DEPLOY_TEST_EXTEND_SECONDS
            ? IntakeConstants.EXTENDED_POSITION_ROTATIONS
            : IntakeConstants.RETRACTED_POSITION_ROTATIONS;
        if (teleopEnabled && !pitCoastMode && isReady()) {
            io.setDeployPosition(target);
        } else {
            io.stop();
        }
        io.setRollerRPM(0.0);
        SmartDashboard.putBoolean("Intake/Ready", isReady());
        SmartDashboard.putBoolean("Intake/DeployConnected", inputs.deployConnected);
        SmartDashboard.putString("Intake/DeployConfigurationStatus", inputs.deployConfigurationStatus);
        SmartDashboard.putString("Intake/ZeroStatus", inputs.zeroStatus);
        SmartDashboard.putNumber("Intake/DeployPositionRotations", inputs.deployPositionRotations);
        SmartDashboard.putNumber("Intake/DeployTargetRotations", target);
        Logger.recordOutput("Intake/DeployTargetRotations", target);
        Logger.recordOutput("Intake/State", getState().toString());
    }
}
