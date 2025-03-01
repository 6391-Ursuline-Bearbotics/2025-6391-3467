package frc.robot.subsystems.Claw.IntakeDS;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.GenericDigitalSensorSubsystem.GenericDigitalSensorSubsystem;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

public class IntakeDS extends GenericDigitalSensorSubsystem {

    public Trigger triggered = new Trigger(() -> super.isTriggered());

    public IntakeDS(IntakeDSIO io)
    {
        super(IntakeDSConstants.kSubSysConstants.kName, io);
    }

}
