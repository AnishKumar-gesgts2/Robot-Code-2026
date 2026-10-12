package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.lang.reflect.Field;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;

class RobotContainerDriverControlsTest {
    @BeforeAll
    static void initializeHal() {
        assertTrue(HAL.initialize(500, 0));
    }

    @AfterEach
    void resetState() {
        CommandScheduler.getInstance().cancelAll();
        DriverStationSim.resetData();
        DriverStationSim.notifyNewData();
    }

    @Test
    void driverRightTriggerRunsFixedShotAndReleaseReturnsToIdle() throws Exception {
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setEnabled(true);
        DriverStationSim.setAutonomous(false);
        DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
        DriverStationSim.setJoystickAxisCount(0, 6);
        DriverStationSim.setJoystickButtonCount(0, 10);
        DriverStationSim.setJoystickIsXbox(0, true);
        DriverStationSim.setJoystickAxis(0, XboxController.Axis.kRightTrigger.value, 0.0);
        DriverStationSim.notifyNewData();

        RobotContainer container = new RobotContainer();
        // The default field origin is just outside the calibrated HUB range. An ungated driver
        // shot must use the nearest setpoint instead of idling the wheel.
        container.getDrivebase().resetOdometry(new Pose2d(0.0, 0.0, new Rotation2d()));
        CommandScheduler scheduler = CommandScheduler.getInstance();
        scheduler.run();

        Shooter shooter = getField(container, "shooter", Shooter.class);
        Turret turret = getField(container, "turret", Turret.class);
        scheduler.run();
        double turretGoalBeforeTrigger = turret.getGoalDegrees();

        DriverStationSim.setJoystickAxis(0, XboxController.Axis.kRightTrigger.value, 1.0);
        DriverStationSim.notifyNewData();
        for (int i = 0; i < 50; i++) {
            scheduler.run();
        }

        Indexer indexer = getField(container, "indexer", Indexer.class);
        assertNotNull(indexer);
        assertEquals(1.0, indexer.getCommandedSpeed(), 1e-9);
        assertEquals(Constants.ShooterConstants.FIXED_SHOT_RPM, shooter.getGoalRPM(), 1e-6);
        assertTrue(shooter.getCurrentRPM() > 0.0);
        assertEquals(turretGoalBeforeTrigger, turret.getGoalDegrees(), 0.01);

        DriverStationSim.setJoystickAxis(0, XboxController.Axis.kRightTrigger.value, 0.0);
        DriverStationSim.notifyNewData();
        scheduler.run();

        assertEquals(0.0, indexer.getCommandedSpeed(), 1e-9);
        assertEquals(Constants.ShooterConstants.FLYWHEEL_IDLE_RPM, shooter.getGoalRPM(), 1e-6);
    }

    private static <T> T getField(RobotContainer container, String name, Class<T> type)
            throws ReflectiveOperationException {
        Field field = RobotContainer.class.getDeclaredField(name);
        field.setAccessible(true);
        return type.cast(field.get(container));
    }
}
