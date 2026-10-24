package frc.robot.subsystems.intake;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.IntakeConstants;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.AfterEach;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;

class IntakeTest {
    private static class FakeIO implements IntakeIO {
        double deployTarget;
        double rollerTarget;
        boolean stopped;
        boolean ready = true;
        int zeroCalls;
        double position;

        @Override
        public void updateInputs(IntakeIOInputs inputs) {
            inputs.deployConnected = ready;
            inputs.rollerConnected = ready;
            inputs.configured = ready;
            inputs.deployHomed = ready;
            inputs.deployPositionRotations = position;
        }

        @Override
        public void zeroRetracted() {
            zeroCalls++;
        }

        @Override
        public void setDeployPosition(double rotations) {
            deployTarget = rotations;
            stopped = false;
        }

        @Override
        public void setRollerRPM(double rpm) {
            rollerTarget = rpm;
        }

        @Override
        public void stop() {
            stopped = true;
            rollerTarget = 0;
        }
    }

    @BeforeEach
    void enableRobot() {
        assertTrue(HAL.initialize(500, 0));
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
    }

    @AfterEach
    void resetDriverStation() {
        DriverStationSim.resetData();
        DriverStationSim.notifyNewData();
    }

    @Test
    void disabledAndFaultedIntakeStopsAndDoesNotResumeRollers() {
        FakeIO io = new FakeIO();
        Intake intake = new Intake(io);
        try {
            intake.activate();
            intake.periodic();
            assertEquals(IntakeConstants.ROLLER_RPM, io.rollerTarget);
            DriverStationSim.setEnabled(false);
            DriverStationSim.notifyNewData();
            intake.periodic();
            assertTrue(io.stopped);
            assertEquals(Intake.State.EXTENDED_STOPPED, intake.getState());
            DriverStationSim.setEnabled(true);
            DriverStationSim.notifyNewData();
            intake.periodic();
            assertEquals(0.0, io.rollerTarget);
            intake.activate();
            io.ready = false;
            intake.periodic();
            assertTrue(io.stopped);
            assertFalse(intake.atDeployTarget());
            assertFalse(intake.isReady());
        } finally {
            CommandScheduler.getInstance().unregisterSubsystem(intake);
        }
    }

    @Test
    void zeroRequiresDisabledAndBrakeMode() {
        FakeIO io = new FakeIO();
        Intake intake = new Intake(io);
        try {
            intake.zeroRetracted();
            assertEquals(0, io.zeroCalls);
            DriverStationSim.setEnabled(false);
            DriverStationSim.notifyNewData();
            intake.setPitCoastMode(true);
            intake.zeroRetracted();
            assertEquals(0, io.zeroCalls);
            intake.setPitCoastMode(false);
            intake.zeroRetracted();
            assertEquals(1, io.zeroCalls);
        } finally {
            CommandScheduler.getInstance().unregisterSubsystem(intake);
        }
    }

    @Test
    void oscillationReachesBothEndpointsBeforeReversingAndRestoresState() {
        FakeIO io = new FakeIO();
        Intake intake = new Intake(io);
        SimHooks.pauseTiming();
        try {
            intake.activate();
            intake.startShootingOscillation();
            intake.periodic();
            SimHooks.stepTiming(1.0);
            intake.periodic();
            assertEquals(IntakeConstants.SHOOT_OSCILLATION_FORWARD_POSITION_ROTATIONS, io.deployTarget);
            io.position = io.deployTarget;
            intake.periodic();
            SimHooks.stepTiming(0.3);
            intake.periodic();
            assertEquals(IntakeConstants.SHOOT_OSCILLATION_REAR_POSITION_ROTATIONS, io.deployTarget);
            SimHooks.stepTiming(1.0);
            intake.periodic();
            assertEquals(IntakeConstants.SHOOT_OSCILLATION_REAR_POSITION_ROTATIONS, io.deployTarget);
            io.position = io.deployTarget;
            intake.periodic();
            SimHooks.stepTiming(0.3);
            intake.periodic();
            assertEquals(IntakeConstants.SHOOT_OSCILLATION_FORWARD_POSITION_ROTATIONS, io.deployTarget);
            intake.stopShootingOscillation();
            intake.periodic();
            assertEquals(IntakeConstants.EXTENDED_POSITION_ROTATIONS, io.deployTarget);
            assertEquals(IntakeConstants.ROLLER_RPM, io.rollerTarget);
        } finally {
            SimHooks.resumeTiming();
            CommandScheduler.getInstance().unregisterSubsystem(intake);
        }
    }

    @Test
    void simulationStopDoesNotReapplyPositionVoltage() {
        IntakeIOSim io = new IntakeIOSim();
        IntakeIO.IntakeIOInputs inputs = new IntakeIO.IntakeIOInputs();
        io.setDeployPosition(IntakeConstants.EXTENDED_POSITION_ROTATIONS);
        io.updateInputs(inputs);
        assertTrue(inputs.deployAppliedVolts > 0.0);
        io.stop();
        io.updateInputs(inputs);
        assertEquals(0.0, inputs.deployAppliedVolts);
        io.setCoastMode(true);
        io.updateInputs(inputs);
        assertFalse(inputs.deployHomed);
        io.setCoastMode(false);
        io.zeroRetracted();
        io.updateInputs(inputs);
        assertTrue(inputs.deployHomed);
    }

    @Test
    void deployRollOscillateAndRetract() {
        assertTrue(HAL.initialize(500, 0));
        FakeIO io = new FakeIO();
        Intake intake = new Intake(io);
        try {
            intake.activate();
            intake.periodic();
            assertEquals(IntakeConstants.EXTENDED_POSITION_ROTATIONS, io.deployTarget);
            assertEquals(IntakeConstants.ROLLER_RPM, io.rollerTarget);

            intake.setExtendedStopped();
            intake.periodic();
            assertEquals(IntakeConstants.EXTENDED_POSITION_ROTATIONS, io.deployTarget);
            assertEquals(0, io.rollerTarget);

            intake.startShootingOscillation();
            intake.periodic();
            assertTrue(intake.isShootingOscillationActive());
            assertEquals(IntakeConstants.SHOOT_OSCILLATION_FORWARD_POSITION_ROTATIONS, io.deployTarget);
            assertEquals(0, io.rollerTarget);
            intake.stopShootingOscillation();

            intake.activateOuttake();
            intake.periodic();
            assertEquals(-IntakeConstants.ROLLER_RPM, io.rollerTarget);

            intake.deactivate();
            intake.periodic();
            assertEquals(IntakeConstants.RETRACTED_POSITION_ROTATIONS, io.deployTarget);
            assertEquals(0, io.rollerTarget);

            intake.setPitCoastMode(true);
            intake.periodic();
            assertTrue(io.stopped);
            assertFalse(intake.isShootingOscillationActive());
        } finally {
            CommandScheduler.getInstance().unregisterSubsystem(intake);
        }
    }
}
