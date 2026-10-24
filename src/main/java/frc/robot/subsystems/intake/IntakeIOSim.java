package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.IntakeConstants;

public class IntakeIOSim implements IntakeIO {
    private final DCMotorSim deploySim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX44(1), 0.01, 1.0), DCMotor.getKrakenX44(1));
    private final DCMotorSim rollerSim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.001, 1.0), DCMotor.getKrakenX60(1));
    private double deployTargetRotations = IntakeConstants.RETRACTED_POSITION_ROTATIONS;
    private double deployAppliedVolts;
    private double rollerAppliedVolts;
    private boolean deployClosedLoop;
    private boolean coastMode;
    private boolean homed = true;

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        double positionError = deployTargetRotations - deploySim.getAngularPositionRotations();
        deployAppliedVolts = deployClosedLoop && !coastMode
            ? MathUtil.clamp(positionError * 4.0, -12.0, 12.0) : 0.0;
        deploySim.setInputVoltage(deployAppliedVolts);
        rollerSim.setInputVoltage(rollerAppliedVolts);
        deploySim.update(0.02);
        rollerSim.update(0.02);
        // Match the hardware IO units: roller feedback is motor RPS.
        inputs.deployConnected = true;
        inputs.rollerConnected = true;
        inputs.configured = true;
        inputs.deployHomed = homed;
        inputs.deployConfigurationStatus = "Simulation";
        inputs.rollerConfigurationStatus = "Simulation";
        inputs.zeroStatus = homed ? "OK" : "Re-zero while disabled";
        inputs.deployPositionRotations = deploySim.getAngularPositionRotations();
        inputs.deployVelocityRPS = deploySim.getAngularVelocityRPM() / 60.0;
        inputs.deployAppliedVolts = deployAppliedVolts;
        inputs.deployCurrentAmps = Math.abs(deploySim.getCurrentDrawAmps());
        inputs.deployTempCelsius = 25.0;
        inputs.rollerVelocityRPS = rollerSim.getAngularVelocityRPM() / 60.0;
        inputs.rollerAppliedVolts = rollerAppliedVolts;
        inputs.rollerCurrentAmps = Math.abs(rollerSim.getCurrentDrawAmps());
        inputs.rollerTempCelsius = 25.0;
    }

    @Override
    public void setRollerRPM(double rpm) {
        rollerAppliedVolts = MathUtil.clamp(
            rpm * IntakeConstants.ROLLER_GEARING / 6000.0 * 12.0, -12.0, 12.0);
    }

    @Override
    public void setDeployPosition(double motorRotations) {
        deployClosedLoop = true;
        deployTargetRotations = MathUtil.clamp(motorRotations,
            IntakeConstants.RETRACTED_POSITION_ROTATIONS, IntakeConstants.EXTENDED_POSITION_ROTATIONS);
    }

    @Override
    public void stop() {
        deployClosedLoop = false;
        deployAppliedVolts = 0.0;
        rollerAppliedVolts = 0.0;
        deploySim.setInputVoltage(0.0);
        rollerSim.setInputVoltage(0.0);
    }

    @Override
    public void zeroRetracted() {
        deploySim.setState(IntakeConstants.RETRACTED_POSITION_ROTATIONS * 2.0 * Math.PI, 0.0);
        deployTargetRotations = IntakeConstants.RETRACTED_POSITION_ROTATIONS;
        homed = true;
    }

    @Override
    public void setCoastMode(boolean coast) {
        coastMode = coast;
        if (coast) {
            homed = false;
        }
    }
}
