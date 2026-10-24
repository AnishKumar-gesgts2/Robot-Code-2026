package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import frc.robot.Constants.IntakeConstants;

/** Only the X44 rack controller is instantiated. The roller is not used. */
public class IntakeIOTalonFX implements IntakeIO {
    private final TalonFX motor = new TalonFX(IntakeConstants.DEPLOY_MOTOR_ID,
        new CANBus(IntakeConstants.CAN_BUS));
    private final MotionMagicVoltage request = new MotionMagicVoltage(0).withSlot(0);
    private final StatusCode configurationStatus;
    private boolean homed;
    private String zeroStatus;
    private String controlStatus = "No request";
    private double lastPrintedTarget = Double.NaN;
    private String lastPrintedControlStatus = "";
    private boolean stopped = true;

    public IntakeIOTalonFX() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = IntakeConstants.DEPLOY_MOTOR_INVERTED
            ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = IntakeConstants.DEPLOY_SUPPLY_CURRENT_LIMIT;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = IntakeConstants.DEPLOY_STATOR_CURRENT_LIMIT;
        config.Slot0.kP = IntakeConstants.DEPLOY_kP;
        config.Slot0.kV = IntakeConstants.DEPLOY_kV;
        config.MotionMagic.MotionMagicCruiseVelocity = IntakeConstants.DEPLOY_CRUISE_VELOCITY_RPS;
        config.MotionMagic.MotionMagicAcceleration = IntakeConstants.DEPLOY_ACCELERATION_RPS_SQ;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = IntakeConstants.RETRACTED_POSITION_ROTATIONS;
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = IntakeConstants.EXTENDED_POSITION_ROTATIONS;
        StatusCode status = motor.getConfigurator().apply(config, 0.1);
        for (int attempt = 1; attempt < 3 && !status.isOK(); attempt++) {
            status = motor.getConfigurator().apply(config, 0.1);
        }
        configurationStatus = status;
        // Hardware-test assumption: rack is physically fully retracted at code startup.
        StatusCode zero = status.isOK()
            ? motor.setPosition(IntakeConstants.RETRACTED_POSITION_ROTATIONS) : status;
        homed = status.isOK() && zero.isOK();
        zeroStatus = zero.toString();
        System.out.printf("[Intake motor] STARTUP bus=%s id=%d configuration=%s zero=%s homed=%s%n",
            IntakeConstants.CAN_BUS, IntakeConstants.DEPLOY_MOTOR_ID,
            configurationStatus, zeroStatus, homed);
        motor.hasResetOccurred();
        BaseStatusSignal.setUpdateFrequencyForAll(50.0, motor.getPosition(), motor.getVelocity(),
            motor.getMotorVoltage(), motor.getSupplyCurrent(), motor.getDeviceTemp());
        motor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        // Only motion feedback determines connection readiness; stale temperature
        // or current telemetry must not suppress an otherwise valid position command.
        boolean fresh = BaseStatusSignal.refreshAll(motor.getPosition(), motor.getVelocity()).isOK();
        BaseStatusSignal.refreshAll(motor.getMotorVoltage(), motor.getSupplyCurrent(), motor.getDeviceTemp());
        if (motor.hasResetOccurred()) {
            homed = false;
            zeroStatus = "Rack controller reset: physically retract and restart code";
        }
        inputs.deployConnected = fresh && motor.isConnected();
        inputs.configured = configurationStatus.isOK();
        inputs.deployHomed = homed;
        inputs.deployConfigurationStatus = configurationStatus.toString();
        inputs.zeroStatus = zeroStatus;
        inputs.deployControlStatus = controlStatus;
        inputs.deployPositionRotations = motor.getPosition().getValueAsDouble();
        inputs.deployVelocityRPS = motor.getVelocity().getValueAsDouble();
        inputs.deployAppliedVolts = motor.getMotorVoltage().getValueAsDouble();
        inputs.deployCurrentAmps = motor.getSupplyCurrent().getValueAsDouble();
        inputs.deployTempCelsius = motor.getDeviceTemp().getValueAsDouble();
    }

    @Override
    public void setDeployPosition(double rotations) {
        if (homed && configurationStatus.isOK() && Double.isFinite(rotations)) {
            double commandedTarget = MathUtil.clamp(rotations,
                IntakeConstants.RETRACTED_POSITION_ROTATIONS, IntakeConstants.EXTENDED_POSITION_ROTATIONS);
            controlStatus = motor.setControl(request.withPosition(commandedTarget)).toString();
            if (stopped || commandedTarget != lastPrintedTarget
                    || !controlStatus.equals(lastPrintedControlStatus)) {
                System.out.printf("[Intake motor] SENT MotionMagic position=%.3f rot bus=%s id=%d setControl=%s%n",
                    commandedTarget, IntakeConstants.CAN_BUS, IntakeConstants.DEPLOY_MOTOR_ID, controlStatus);
                lastPrintedTarget = commandedTarget;
                lastPrintedControlStatus = controlStatus;
            }
            stopped = false;
        } else {
            stop();
        }
    }

    @Override
    public void stop() {
        motor.stopMotor();
        if (!stopped) {
            System.out.printf("[Intake motor] STOP sent bus=%s id=%d%n",
                IntakeConstants.CAN_BUS, IntakeConstants.DEPLOY_MOTOR_ID);
        }
        stopped = true;
    }

    @Override
    public void setCoastMode(boolean coast) {
        motor.setNeutralMode(coast ? NeutralModeValue.Coast : NeutralModeValue.Brake);
    }
}
