package frc.robot.subsystems.intake;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.IntakeConstants;
import org.junit.jupiter.api.Test;

class IntakeTest {
    private static class FakeIO implements IntakeIO {
        double deployTarget;
        double rollerTarget;
        boolean stopped;

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
