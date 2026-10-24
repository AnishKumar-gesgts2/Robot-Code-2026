package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.math.MathUtil;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.IntakeConstants;

/** Hardware implementation for one X44 rack motor and one X60 roller motor. */
public class IntakeIOTalonFX implements IntakeIO {
    private final TalonFX deployMotor = new TalonFX(IntakeConstants.DEPLOY_MOTOR_ID, IntakeConstants.CAN_BUS);
    private final TalonFX rollerMotor = new TalonFX(IntakeConstants.ROLLER_MOTOR_ID, IntakeConstants.CAN_BUS);
    private final MotionMagicVoltage deployRequest = new MotionMagicVoltage(0).withSlot(0);
    private final VelocityVoltage rollerRequest = new VelocityVoltage(0).withSlot(0);
    private StatusCode deployConfigurationStatus;
    private StatusCode rollerConfigurationStatus;
    private String zeroStatus = "Not zeroed: physically retract rack, then zero while disabled";
    private boolean homed;
    private boolean coastMode;

    private final StatusSignal<Angle> deployPosition = deployMotor.getPosition();
    private final StatusSignal<AngularVelocity> deployVelocity = deployMotor.getVelocity();
    private final StatusSignal<Voltage> deployVoltage = deployMotor.getMotorVoltage();
    private final StatusSignal<Current> deployCurrent = deployMotor.getSupplyCurrent();
    private final StatusSignal<Temperature> deployTemp = deployMotor.getDeviceTemp();
    private final StatusSignal<AngularVelocity> rollerVelocity = rollerMotor.getVelocity();
    private final StatusSignal<Voltage> rollerVoltage = rollerMotor.getMotorVoltage();
    private final StatusSignal<Current> rollerCurrent = rollerMotor.getSupplyCurrent();
    private final StatusSignal<Temperature> rollerTemp = rollerMotor.getDeviceTemp();

    public IntakeIOTalonFX() {
        configureDeployMotor();
        configureRollerMotor();
        // A relative encoder cannot establish rack position on boot. Require an explicit zero.
        BaseStatusSignal.setUpdateFrequencyForAll(50.0,
            deployPosition, deployVelocity, deployVoltage, deployCurrent, deployTemp,
            rollerVelocity, rollerVoltage, rollerCurrent, rollerTemp);
        deployMotor.optimizeBusUtilization();
        rollerMotor.optimizeBusUtilization();
    }

    private void configureDeployMotor() {
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
        deployConfigurationStatus = applyWithRetry(deployMotor, config);
    }

    private void configureRollerMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted = IntakeConstants.ROLLER_MOTOR_INVERTED
            ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = IntakeConstants.ROLLER_SUPPLY_CURRENT_LIMIT;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = IntakeConstants.ROLLER_STATOR_CURRENT_LIMIT;
        config.Slot0.kP = IntakeConstants.ROLLER_kP;
        config.Slot0.kV = IntakeConstants.ROLLER_kV;
        rollerConfigurationStatus = applyWithRetry(rollerMotor, config);
    }

    private static StatusCode applyWithRetry(TalonFX motor, TalonFXConfiguration config) {
        StatusCode status = motor.getConfigurator().apply(config, 0.1);
        for (int attempt = 1; attempt < 3 && !status.isOK(); attempt++) {
            status = motor.getConfigurator().apply(config, 0.1);
        }
        return status;
    }

    @Override
    public void zeroRetracted() {
        if (coastMode || !deployConfigurationStatus.isOK() || !rollerConfigurationStatus.isOK()) {
            return;
        }
        // Consume an already-observed reset before setting the new reference.
        deployMotor.hasResetOccurred();
        StatusCode status = deployMotor.setPosition(IntakeConstants.RETRACTED_POSITION_ROTATIONS);
        homed = status.isOK();
        zeroStatus = status.toString();
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        boolean deployFresh = BaseStatusSignal.refreshAll(deployPosition, deployVelocity,
            deployVoltage, deployCurrent, deployTemp).isOK();
        boolean rollerFresh = BaseStatusSignal.refreshAll(rollerVelocity, rollerVoltage,
            rollerCurrent, rollerTemp).isOK();
        if (deployMotor.hasResetOccurred() || !deployFresh) {
            homed = false;
            zeroStatus = "Rack reset or feedback lost: re-zero while disabled";
        }
        inputs.deployConnected = deployFresh && deployMotor.isConnected();
        inputs.rollerConnected = rollerFresh && rollerMotor.isConnected();
        inputs.configured = deployConfigurationStatus.isOK() && rollerConfigurationStatus.isOK();
        inputs.deployHomed = homed;
        inputs.deployConfigurationStatus = deployConfigurationStatus.toString();
        inputs.rollerConfigurationStatus = rollerConfigurationStatus.toString();
        inputs.zeroStatus = zeroStatus;
        inputs.deployPositionRotations = deployPosition.getValueAsDouble();
        inputs.deployVelocityRPS = deployVelocity.getValueAsDouble();
        inputs.deployAppliedVolts = deployVoltage.getValueAsDouble();
        inputs.deployCurrentAmps = deployCurrent.getValueAsDouble();
        inputs.deployTempCelsius = deployTemp.getValueAsDouble();
        inputs.rollerVelocityRPS = rollerVelocity.getValueAsDouble();
        inputs.rollerAppliedVolts = rollerVoltage.getValueAsDouble();
        inputs.rollerCurrentAmps = rollerCurrent.getValueAsDouble();
        inputs.rollerTempCelsius = rollerTemp.getValueAsDouble();
    }

    @Override
    public void setRollerRPM(double rpm) {
        if (rpm == 0.0) {
            rollerMotor.stopMotor();
        } else {
            rollerMotor.setControl(rollerRequest.withVelocity(rpm / 60.0 * IntakeConstants.ROLLER_GEARING));
        }
    }

    @Override
    public void setDeployPosition(double motorRotations) {
        if (homed && !coastMode && Double.isFinite(motorRotations)) {
            deployMotor.setControl(deployRequest.withPosition(MathUtil.clamp(motorRotations,
                IntakeConstants.RETRACTED_POSITION_ROTATIONS, IntakeConstants.EXTENDED_POSITION_ROTATIONS)));
        } else {
            deployMotor.stopMotor();
        }
    }

    @Override
    public void stop() {
        deployMotor.stopMotor();
        rollerMotor.stopMotor();
    }

    @Override
    public void setCoastMode(boolean coast) {
        coastMode = coast;
        if (coast) {
            homed = false;
            zeroStatus = "Rack moved in coast: re-zero while disabled";
        }
        deployMotor.setNeutralMode(coast ? NeutralModeValue.Coast : NeutralModeValue.Brake);
        rollerMotor.setNeutralMode(NeutralModeValue.Coast);
    }
}
