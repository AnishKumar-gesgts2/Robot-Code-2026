package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.Logger;

public class IndexerIOTalonFX implements IndexerIO {
    private static final double FLOOR_GEAR_RATIO = 2.5;
    private static final double TUNNEL_GEAR_RATIO = 5.0 / 3.0;
    private static final double CURRENT_LIMIT_AMPS = 40.0;
    private static final double STATUS_FREQUENCY_HZ = 50.0;
    private static final String CAN_BUS = "Canivore";

    private final TalonFX floorLeader;
    private final TalonFX tunnelMotor;

    private final DutyCycleOut floorRequest = new DutyCycleOut(0.0);
    private final DutyCycleOut tunnelRequest = new DutyCycleOut(0.0);

    private final StatusSignal<Angle> floorPosition;
    private final StatusSignal<AngularVelocity> floorVelocity;
    private final StatusSignal<Voltage> floorVoltage;
    private final StatusSignal<Current> floorLeaderCurrent;

    private final StatusSignal<Angle> tunnelPosition;
    private final StatusSignal<AngularVelocity> tunnelVelocity;
    private final StatusSignal<Voltage> tunnelVoltage;
    private final StatusSignal<Current> tunnelCurrent;

    public IndexerIOTalonFX(
            int floorLeaderId,
            int tunnelMotorId) {
        this(floorLeaderId, tunnelMotorId, new CANBus(CAN_BUS));
    }

    public IndexerIOTalonFX(
            int floorLeaderId,
            int tunnelMotorId,
            CANBus canBus) {
        floorLeader = new TalonFX(floorLeaderId, canBus);
        tunnelMotor = new TalonFX(tunnelMotorId, canBus);

        configureMotor(floorLeader, FLOOR_GEAR_RATIO);
        configureMotor(tunnelMotor, TUNNEL_GEAR_RATIO);

        floorPosition = floorLeader.getPosition();
        floorVelocity = floorLeader.getVelocity();
        floorVoltage = floorLeader.getMotorVoltage();
        floorLeaderCurrent = floorLeader.getStatorCurrent();

        tunnelPosition = tunnelMotor.getPosition();
        tunnelVelocity = tunnelMotor.getVelocity();
        tunnelVoltage = tunnelMotor.getMotorVoltage();
        tunnelCurrent = tunnelMotor.getStatorCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(
                STATUS_FREQUENCY_HZ,
                floorPosition,
                floorVelocity,
                floorVoltage,
                floorLeaderCurrent,
                tunnelPosition,
                tunnelVelocity,
                tunnelVoltage,
                tunnelCurrent);

        floorLeader.optimizeBusUtilization();
        tunnelMotor.optimizeBusUtilization();
    }

    private static void configureMotor(TalonFX motor, double gearRatio) {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Feedback.SensorToMechanismRatio = gearRatio;
        config.CurrentLimits.StatorCurrentLimit = CURRENT_LIMIT_AMPS;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT_AMPS;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        motor.getConfigurator().apply(config);
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                floorPosition,
                floorVelocity,
                floorVoltage,
                floorLeaderCurrent,
                tunnelPosition,
                tunnelVelocity,
                tunnelVoltage,
                tunnelCurrent);

        inputs.appliedVolts = tunnelVoltage.getValueAsDouble();
        inputs.currentAmps = floorLeaderCurrent.getValueAsDouble() + tunnelCurrent.getValueAsDouble();

        Logger.recordOutput("Indexer/Connected/FloorLeader", floorLeader.isConnected());
        Logger.recordOutput("Indexer/Connected/TunnelMotor", tunnelMotor.isConnected());
    }

    @Override
    public void setSpeed(double dutyCycle) {
        floorLeader.setControl(floorRequest.withOutput(dutyCycle));
        tunnelMotor.setControl(tunnelRequest.withOutput(dutyCycle));
    }

    @Override
    public void stop() {
        floorLeader.setControl(floorRequest.withOutput(0.0));
        tunnelMotor.setControl(tunnelRequest.withOutput(0.0));
    }
}
