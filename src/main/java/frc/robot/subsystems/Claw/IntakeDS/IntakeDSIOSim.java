package frc.robot.subsystems.Claw.IntakeDS;

import frc.robot.subsystems.GenericDigitalSensorSubsystem.GenericDigitalSensorSubsystemIOImpl;

public class IntakeDSIOSim extends GenericDigitalSensorSubsystemIOImpl
    implements IntakeDSIO {

    public IntakeDSIOSim()
    {
        super(IntakeDSConstants.kSubSysConstants, true);
    }
}
