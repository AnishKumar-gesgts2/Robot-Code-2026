package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.IntakeConstants;

/** Simple rack-only simulation using motor-rotation units. */
public class IntakeIOSim implements IntakeIO {
    private final DCMotorSim deploySim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX44(1), 0.01, 1.0),
        DCMotor.getKrakenX44(1));
    private double target;
    private boolean closedLoop;
    private boolean coastMode;

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        double volts = closedLoop && !coastMode
            ? MathUtil.clamp((target - deploySim.getAngularPositionRotations()) * 4.0, -12.0, 12.0)
            : 0.0;
        deploySim.setInputVoltage(volts);
        deploySim.update(0.02);
        inputs.deployConnected = true;
        inputs.configured = true;
        inputs.deployHomed = true; // Starts retracted, like the hardware-test startup assumption.
        inputs.deployConfigurationStatus = "Simulation";
        inputs.zeroStatus = "Simulation: started retracted";
        inputs.deployPositionRotations = deploySim.getAngularPositionRotations();
        inputs.deployVelocityRPS = deploySim.getAngularVelocityRPM() / 60.0;
        inputs.deployAppliedVolts = volts;
        inputs.deployCurrentAmps = Math.abs(deploySim.getCurrentDrawAmps());
        inputs.deployTempCelsius = 25.0;
    }

    @Override
    public void setDeployPosition(double rotations) {
        target = MathUtil.clamp(rotations, IntakeConstants.RETRACTED_POSITION_ROTATIONS,
            IntakeConstants.EXTENDED_POSITION_ROTATIONS);
        closedLoop = true;
    }

    @Override
    public void stop() {
        closedLoop = false;
        deploySim.setInputVoltage(0.0);
    }

    @Override
    public void setCoastMode(boolean coast) { coastMode = coast; }
}
