package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.Logger;
import frc.robot.Constants.IndexerConstants;

public class IndexerIOTalonFX implements IndexerIO {
    private static final double CURRENT_LIMIT_AMPS = 40.0;
    private static final double STATUS_FREQUENCY_HZ = 50.0;
    private static final String CAN_BUS = "Canivore";

    private final TalonFX floorLeader;
    private final TalonFX tunnelMotor;

    private final VelocityVoltage floorRequest = new VelocityVoltage(0.0);
    private final VelocityVoltage tunnelRequest = new VelocityVoltage(0.0);

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

        configureMotor(floorLeader, IndexerConstants.FLOOR_GEAR_RATIO);
        configureMotor(tunnelMotor, IndexerConstants.TUNNEL_GEAR_RATIO);

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
        config.Slot0.kP = IndexerConstants.VELOCITY_KP;
        config.Slot0.kV = IndexerConstants.MOTOR_KV * gearRatio;
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
        inputs.floorRPM = floorVelocity.getValueAsDouble() * 60.0;
        inputs.tunnelRPM = tunnelVelocity.getValueAsDouble() * 60.0;
        inputs.currentAmps = floorLeaderCurrent.getValueAsDouble() + tunnelCurrent.getValueAsDouble();

        Logger.recordOutput("Indexer/Connected/FloorLeader", floorLeader.isConnected());
        Logger.recordOutput("Indexer/Connected/TunnelMotor", tunnelMotor.isConnected());
    }

    @Override
    public void setFloorRPM(double rpm) {
        if (rpm == 0.0) {
            floorLeader.stopMotor();
        } else {
            floorLeader.setControl(floorRequest.withVelocity(rpm / 60.0));
        }
    }

    @Override
    public void setTunnelRPM(double rpm) {
        if (rpm == 0.0) {
            tunnelMotor.stopMotor();
        } else {
            tunnelMotor.setControl(tunnelRequest.withVelocity(rpm / 60.0));
        }
    }

    @Override
    public void stop() {
        floorLeader.stopMotor();
        tunnelMotor.stopMotor();
    }
}
