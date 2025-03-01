package frc.robot.subsystems.Claw.IntakeDS;

import frc.robot.subsystems.GenericDigitalSensorSubsystem.GenericDigitalSensorSubsystemIOImpl;

public class IntakeDSIOReal extends GenericDigitalSensorSubsystemIOImpl
    implements IntakeDSIO {

    public IntakeDSIOReal()
    {
        super(IntakeDSConstants.kSubSysConstants, false);
    }
}
