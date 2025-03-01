package frc.robot.subsystems.Claw.IntakeDS;

import frc.robot.Ports;
import frc.robot.subsystems.GenericDigitalSensorSubsystem.GenericDigitalSensorSubsystemConstants;

public class IntakeDSConstants {
    public static final GenericDigitalSensorSubsystemConstants kSubSysConstants =
        new GenericDigitalSensorSubsystemConstants();

    static {
        kSubSysConstants.kName = "IntakeDS";
        kSubSysConstants.digitalPort = Ports.RAMP_DIGITAL;
    }
}
