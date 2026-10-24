package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    public enum State {
        RETRACTED_STOPPED,
        EXTENDED_STOPPED,
        EXTENDED_SPINNING,
        EXTENDED_OUTTAKING
    }

    private State state = State.RETRACTED_STOPPED;
    private boolean shootingOscillation = false;
    private double oscillationAtTargetSince = Double.NaN;
    private boolean oscillationForward = true;
    private boolean pitCoastMode = false;

    public Intake(IntakeIO io) {
        this.io = io;
    }

    public void activate() {
        setExtendedSpinning();
    }

    public void deactivate() {
        setRetracted();
    }

    /** Preserve the existing Driver 1 outtake command with the new rack intake. */
    public void activateOuttake() {
        setOperatingState(State.EXTENDED_OUTTAKING);
    }

    public void setExtendedSpinning() {
        setOperatingState(State.EXTENDED_SPINNING);
    }

    public void setExtendedStopped() {
        setOperatingState(State.EXTENDED_STOPPED);
    }

    public void setRetracted() {
        setOperatingState(State.RETRACTED_STOPPED);
    }

    private void setOperatingState(State newState) {
        state = newState;
    }

    /** Temporarily oscillates the rack for shooting, then restores the previous intake state. */
    public void startShootingOscillation() {
        if (!shootingOscillation) {
            oscillationAtTargetSince = Double.NaN;
            oscillationForward = true;
            shootingOscillation = true;
        }
    }

    public void stopShootingOscillation() {
        if (shootingOscillation) {
            shootingOscillation = false;
        }
    }

    public State getState() {
        return state;
    }

    public boolean isActive() {
        return state == State.EXTENDED_SPINNING;
    }

    public boolean isShootingOscillationActive() {
        return shootingOscillation;
    }

    public boolean atDeployTarget() {
        return isReady() && !pitCoastMode
            && Math.abs(inputs.deployPositionRotations - getDeployTargetRotations())
            <= IntakeConstants.DEPLOY_POSITION_TOLERANCE_ROTATIONS;
    }

    public boolean isReady() {
        return inputs.configured && inputs.deployHomed
            && inputs.deployConnected && inputs.rollerConnected;
    }

    public void zeroRetracted() {
        if (DriverStation.isDisabled() && !pitCoastMode) {
            shootingOscillation = false;
            state = State.RETRACTED_STOPPED;
            io.stop();
            io.zeroRetracted();
        }
    }

    public void setPitCoastMode(boolean enabled) {
        pitCoastMode = enabled;
        shootingOscillation = false;
        state = State.RETRACTED_STOPPED;
        io.stop();
        io.setCoastMode(enabled);
    }

    private double getDeployTargetRotations() {
        if (shootingOscillation) {
            return oscillationForward
                ? IntakeConstants.SHOOT_OSCILLATION_FORWARD_POSITION_ROTATIONS
                : IntakeConstants.SHOOT_OSCILLATION_REAR_POSITION_ROTATIONS;
        }

        return state == State.RETRACTED_STOPPED
            ? IntakeConstants.RETRACTED_POSITION_ROTATIONS
            : IntakeConstants.EXTENDED_POSITION_ROTATIONS;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);

        SmartDashboard.putBoolean("Intake/Ready", isReady() && !pitCoastMode);
        SmartDashboard.putBoolean("Intake/Homed", inputs.deployHomed);
        SmartDashboard.putBoolean("Intake/DeployConnected", inputs.deployConnected);
        SmartDashboard.putBoolean("Intake/RollerConnected", inputs.rollerConnected);
        SmartDashboard.putString("Intake/DeployConfigurationStatus", inputs.deployConfigurationStatus);
        SmartDashboard.putString("Intake/RollerConfigurationStatus", inputs.rollerConfigurationStatus);
        SmartDashboard.putString("Intake/ZeroStatus", inputs.zeroStatus);
        SmartDashboard.putNumber("Intake/DeployPositionRotations", inputs.deployPositionRotations);
        SmartDashboard.putNumber("Intake/RollerRPM", inputs.rollerVelocityRPS * 60.0
            / IntakeConstants.ROLLER_GEARING);
        Logger.recordOutput("Intake/State", state.toString());
        Logger.recordOutput("Intake/PitCoastMode", pitCoastMode);
        if (DriverStation.isDisabled() || pitCoastMode || !isReady()) {
            shootingOscillation = false;
            // Disabling or losing feedback must not leave rollers latched on for re-enable.
            if (state == State.EXTENDED_SPINNING || state == State.EXTENDED_OUTTAKING) {
                state = State.EXTENDED_STOPPED;
            }
            io.stop();
            Logger.recordOutput("Intake/ShootingOscillation", false);
            Logger.recordOutput("Intake/AtDeployTarget", false);
            return;
        }

        // Wait until each endpoint is actually reached before timing the dwell/reversal.
        // A fixed 0.25 s reversal can turn around before the rack traverses even 2 inches.
        if (shootingOscillation) {
            if (atDeployTarget()) {
                double now = Timer.getFPGATimestamp();
                if (Double.isNaN(oscillationAtTargetSince)) {
                    oscillationAtTargetSince = now;
                } else if (now - oscillationAtTargetSince
                        >= IntakeConstants.SHOOT_OSCILLATION_HALF_PERIOD_SECONDS) {
                    oscillationForward = !oscillationForward;
                    oscillationAtTargetSince = Double.NaN;
                }
            } else {
                oscillationAtTargetSince = Double.NaN;
            }
        }
        double deployTargetRotations = getDeployTargetRotations();
        io.setDeployPosition(deployTargetRotations);
        double rollerRPM = !shootingOscillation && state == State.EXTENDED_SPINNING
            ? IntakeConstants.ROLLER_RPM
            : !shootingOscillation && state == State.EXTENDED_OUTTAKING
                ? -IntakeConstants.ROLLER_RPM
                : 0.0;
        io.setRollerRPM(rollerRPM);

        Logger.recordOutput("Intake/State", state.toString());
        Logger.recordOutput("Intake/ShootingOscillation", shootingOscillation);
        Logger.recordOutput("Intake/DeployTargetRotations", deployTargetRotations);
        Logger.recordOutput("Intake/AtDeployTarget", atDeployTarget());
    }
}
