package frc.robot.subsystems.intake;

import static org.junit.jupiter.api.Assertions.*;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.IntakeConstants;
import org.junit.jupiter.api.*;

class IntakeTest {
    private static class FakeIO implements IntakeIO {
        double target;
        double roller;
        boolean stopped;
        boolean ready = true;
        @Override public void updateInputs(IntakeIOInputs inputs) {
            inputs.deployConnected = ready;
            inputs.configured = ready;
            inputs.deployHomed = ready;
            // No roller is connected or required for the rack-only test.
        }
        @Override public void setDeployPosition(double position) { target = position; stopped = false; }
        @Override public void setRollerRPM(double rpm) { roller = rpm; }
        @Override public void stop() { stopped = true; }
    }
    private FakeIO io;
    private Intake intake;

    @BeforeEach void setup() {
        assertTrue(HAL.initialize(500, 0));
        SimHooks.pauseTiming();
        DriverStationSim.resetData();
        DriverStationSim.setDsAttached(true);
        DriverStationSim.notifyNewData();
        io = new FakeIO();
        intake = new Intake(io);
    }

    @AfterEach void cleanup() {
        CommandScheduler.getInstance().unregisterSubsystem(intake);
        DriverStationSim.resetData();
        DriverStationSim.notifyNewData();
        SimHooks.resumeTiming();
    }

    private void teleop(boolean enabled) {
        DriverStationSim.setAutonomous(false);
        DriverStationSim.setTest(false);
        DriverStationSim.setEnabled(enabled);
        DriverStationSim.notifyNewData();
        intake.periodic();
    }

    @Test void extendsImmediatelyThenRetractsOnceAndHolds() {
        teleop(true);
        assertEquals(IntakeConstants.EXTENDED_POSITION_ROTATIONS, io.target);
        assertFalse(io.stopped);
        assertTrue(intake.isReady());
        SimHooks.stepTiming(IntakeConstants.DEPLOY_TEST_EXTEND_SECONDS - 0.01);
        intake.periodic();
        assertEquals(IntakeConstants.EXTENDED_POSITION_ROTATIONS, io.target);
        SimHooks.stepTiming(0.02);
        intake.periodic();
        assertEquals(IntakeConstants.RETRACTED_POSITION_ROTATIONS, io.target);
        SimHooks.stepTiming(20.0);
        intake.periodic();
        assertEquals(IntakeConstants.RETRACTED_POSITION_ROTATIONS, io.target);
        assertEquals(0.0, io.roller);
    }

    @Test void disableStopsImmediatelyAndNextTeleopEnableRepeatsTest() {
        teleop(true);
        SimHooks.stepTiming(0.5);
        teleop(false);
        assertTrue(io.stopped);
        teleop(true);
        assertFalse(io.stopped);
        assertEquals(IntakeConstants.EXTENDED_POSITION_ROTATIONS, io.target);
        SimHooks.stepTiming(IntakeConstants.DEPLOY_TEST_EXTEND_SECONDS + 0.01);
        intake.periodic();
        assertEquals(IntakeConstants.RETRACTED_POSITION_ROTATIONS, io.target);
    }

    @Test void disabledAutonomousAndTestModesNeverDriveRack() {
        intake.periodic();
        assertTrue(io.stopped);
        DriverStationSim.setEnabled(true);
        DriverStationSim.setAutonomous(true);
        DriverStationSim.notifyNewData();
        intake.periodic();
        assertTrue(io.stopped);
        DriverStationSim.setAutonomous(false);
        DriverStationSim.setTest(true);
        DriverStationSim.notifyNewData();
        intake.periodic();
        assertTrue(io.stopped);
    }

    @Test void legacyIntakeAndShotCallsCannotOverrideHardwareTest() {
        teleop(true);
        intake.activate();
        intake.activateOuttake();
        intake.setRetracted();
        intake.setExtendedStopped();
        intake.startShootingOscillation();
        intake.periodic();
        assertEquals(IntakeConstants.EXTENDED_POSITION_ROTATIONS, io.target);
        assertEquals(0.0, io.roller);
        assertFalse(intake.isActive());
        assertFalse(intake.isShootingOscillationActive());
        SimHooks.stepTiming(IntakeConstants.DEPLOY_TEST_EXTEND_SECONDS + 0.01);
        intake.activate();
        intake.periodic();
        assertEquals(IntakeConstants.RETRACTED_POSITION_ROTATIONS, io.target);
    }

    @Test void feedbackFaultAndPitCoastStopOutput() {
        teleop(true);
        io.ready = false;
        intake.periodic();
        assertTrue(io.stopped);
        assertFalse(intake.isReady());
        io.ready = true;
        intake.setPitCoastMode(true);
        intake.periodic();
        assertTrue(io.stopped);
        intake.setPitCoastMode(false);
        intake.periodic();
        assertFalse(io.stopped);
    }
}
