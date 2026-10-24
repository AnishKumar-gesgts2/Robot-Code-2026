package frc.robot;

import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.assertEquals;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import java.nio.file.Path;
import java.util.HashSet;
import java.util.Set;
import org.junit.jupiter.api.Test;

class HardwareConfigurationTest {
    private final Set<String> devices = new HashSet<>();

    private void add(String bus, String type, int id) {
        assertTrue(id >= 0 && id <= 62, "Invalid CAN ID: " + id);
        assertTrue(devices.add(bus + "/" + type + "/" + id),
            "Duplicate CAN address: " + bus + "/" + type + "/" + id);
    }

    @Test
    void installedMechanismsAndDeployedSwerveHaveUniqueCanAddresses() throws Exception {
        add(Constants.ShooterConstants.CAN_BUS, "talonfx", Constants.ShooterConstants.LEFT_FLYWHEEL_ID);
        add(Constants.ShooterConstants.CAN_BUS, "talonfx", Constants.ShooterConstants.RIGHT_FLYWHEEL_ID);
        add(Constants.IndexerConstants.CAN_BUS, "talonfx", Constants.IndexerConstants.FLOOR_MOTOR_ID);
        add(Constants.IndexerConstants.CAN_BUS, "talonfx", Constants.IndexerConstants.TUNNEL_MOTOR_ID);
        if (Constants.HardwareConstants.RACK_INTAKE_INSTALLED) {
            add(Constants.IntakeConstants.CAN_BUS, "talonfx", Constants.IntakeConstants.DEPLOY_MOTOR_ID);
        }
        if (Constants.HardwareConstants.LEGACY_TURRET_INSTALLED) {
            add("rio", "talonfx", Constants.TurretConstants.MOTOR_ID);
            add("rio", "cancoder", Constants.TurretConstants.ENCODER_ID);
        }
        if (Constants.HardwareConstants.LEGACY_FEEDER_INSTALLED) {
            add(Constants.FeederConstants.CAN_BUS, "talonfx", Constants.FeederConstants.PAN_MOTOR_ID);
            add(Constants.FeederConstants.CAN_BUS, "talonfx", Constants.FeederConstants.PUSHER_MOTOR_ID);
            add(Constants.FeederConstants.CAN_BUS, "talonfx", Constants.FeederConstants.FLOOR_ID);
        }
        ObjectMapper mapper = new ObjectMapper();
        Path root = Path.of("src/main/deploy/swerve/falcon");
        JsonNode swerve = mapper.readTree(root.resolve("swervedrive.json").toFile());
        JsonNode imu = swerve.get("imu");
        add(imu.get("canbus").asText(), imu.get("type").asText(), imu.get("id").asInt());
        for (JsonNode name : swerve.get("modules")) {
            JsonNode module = mapper.readTree(root.resolve("modules").resolve(name.asText()).toFile());
            for (String component : new String[] {"drive", "angle", "encoder"}) {
                JsonNode device = module.get(component);
                String type = device.get("type").asText();
                // X44 and X60 both use the Talon FX device type on CAN.
                if (type.startsWith("kraken") || type.equals("talonfx")) {
                    type = "talonfx";
                }
                add(device.get("canbus").asText(), type, device.get("id").asInt());
            }
        }
    }

    @Test
    void rackTestTargetAndTimingFitConfiguredTravel() {
        assertEquals(26, Constants.IntakeConstants.DEPLOY_MOTOR_ID,
            "Extension motor's confirmed CAN ID");
        assertTrue(Constants.IntakeConstants.RETRACTED_POSITION_ROTATIONS
            < Constants.IntakeConstants.SHOOT_OSCILLATION_REAR_POSITION_ROTATIONS);
        assertTrue(Constants.IntakeConstants.SHOOT_OSCILLATION_REAR_POSITION_ROTATIONS
            < Constants.IntakeConstants.SHOOT_OSCILLATION_FORWARD_POSITION_ROTATIONS);
        assertTrue(Constants.IntakeConstants.SHOOT_OSCILLATION_FORWARD_POSITION_ROTATIONS
            <= Constants.IntakeConstants.EXTENDED_POSITION_ROTATIONS);
        assertTrue(Constants.IntakeConstants.DEPLOY_TEST_EXTEND_SECONDS
            > Constants.IntakeConstants.EXTENDED_POSITION_ROTATIONS
                / Constants.IntakeConstants.DEPLOY_CRUISE_VELOCITY_RPS);
    }
}
