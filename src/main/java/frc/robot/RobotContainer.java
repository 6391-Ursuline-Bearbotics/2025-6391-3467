// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.Vision.VisionConstants.*;
import java.util.function.Supplier;
import com.ctre.phoenix.led.CANdle;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.RobotType;
import frc.robot.FieldConstants.ReefSide;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Arm.*;
import frc.robot.subsystems.Claw.ClawRoller.ClawRoller;
import frc.robot.subsystems.Claw.ClawRoller.ClawRollerIO;
import frc.robot.subsystems.Claw.ClawRoller.ClawRollerIOSim;
import frc.robot.subsystems.Claw.ClawRoller.ClawRollerIOTalonFX;
import frc.robot.subsystems.Claw.ClawRollerDS.ClawRollerDS;
import frc.robot.subsystems.Claw.ClawRollerDS.ClawRollerDSIO;
import frc.robot.subsystems.Claw.ClawRollerDS.ClawRollerDSIOReal;
import frc.robot.subsystems.Claw.ClawRollerDS.ClawRollerDSIOSim;
import frc.robot.subsystems.Claw.IntakeDS.IntakeDS;
import frc.robot.subsystems.Claw.IntakeDS.IntakeDSIO;
import frc.robot.subsystems.Claw.IntakeDS.IntakeDSIOReal;
import frc.robot.subsystems.Claw.IntakeDS.IntakeDSIOSim;
import frc.robot.subsystems.Climber.Climber;
import frc.robot.subsystems.Climber.ClimberIO;
import frc.robot.subsystems.Climber.ClimberIOSim;
import frc.robot.subsystems.Elevator.*;
import frc.robot.subsystems.Vision.*;
import frc.robot.subsystems.drive.*;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.WindupXboxController;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    // Controllers
    private final WindupXboxController m_driver = new WindupXboxController(0);
    private final WindupXboxController m_operator = new WindupXboxController(0);

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> m_autoChooser;

    // Maple Sim
    private SwerveDriveSimulation m_driveSimulation = null;

    // AK-enabled Subsystems
    public final Drive m_drive;
    private final Arm m_profiledArm;
    private final Elevator m_profiledElevator;
    // private final Climber m_profiledClimber;
    private final ClawRoller m_clawRoller;
    private final ClawRollerDS m_ClawRollerDS;
    public final IntakeDS m_IntakeDS;
    private final Superstructure m_superStruct;

    public final Vision m_vision;

    // Trigger for algae/coral mode switching
    private boolean coralModeEnabled = true;
    private Trigger isCoralMode = new Trigger(() -> coralModeEnabled);

    private double speedMultiplier = 0.85;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer()
    {
        switch (Constants.currentMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                m_drive =
                    new Drive(
                        new GyroIOPigeon2(),
                        new ModuleIOTalonFXReal(TunerConstants.FrontLeft),
                        new ModuleIOTalonFXReal(TunerConstants.FrontRight),
                        new ModuleIOTalonFXReal(TunerConstants.BackLeft),
                        new ModuleIOTalonFXReal(TunerConstants.BackRight),
                        (robotPose) -> {
                        });

                m_profiledArm = new Arm(new ArmIOTalonFX(), false);
                m_profiledElevator = new Elevator(new ElevatorIOTalonFX(), false);
                // m_profiledClimber = new Climber(new ClimberIOTalonFX(), false);
                m_clawRoller = new ClawRoller(new ClawRollerIOTalonFX(), false);
                m_ClawRollerDS = new ClawRollerDS(new ClawRollerDSIOReal());
                m_IntakeDS = new IntakeDS(new IntakeDSIOReal());

                m_vision = new Vision(m_drive);

                // break;

                // m_drive =
                // new Drive(
                // new GyroIO() {},
                // new ModuleIO() {},
                // new ModuleIO() {},
                // new ModuleIO() {},
                // new ModuleIO() {},
                // (robotPose) -> {
                // });

                // m_profiledArm = new Arm(new ArmIO() {}, true);
                // m_profiledElevator = new Elevator(new ElevatorIO() {}, true);
                // m_profiledClimber = new Climber(new ClimberIO() {}, true);
                // m_clawRoller = new ClawRoller(new ClawRollerIO() {}, true);
                // m_ClawRollerDS = new ClawRollerDS(new ClawRollerDSIO() {});
                // m_IntakeDS = new IntakeDS(new IntakeDSIO() {});

                // m_vision = new Vision(m_drive, new VisionIO() {}, new VisionIO() {});

                break;

            case SIM:
                // Sim robot, instantiate physics sim IO implementations
                m_driveSimulation =
                    new SwerveDriveSimulation(Drive.mapleSimConfig,
                        new Pose2d(3, 3, new Rotation2d()));
                SimulatedArena.getInstance().addDriveTrainSimulation(m_driveSimulation);
                m_drive =
                    new Drive(
                        new GyroIOSim(this.m_driveSimulation.getGyroSimulation()),
                        new ModuleIOTalonFXSim(
                            TunerConstants.FrontLeft, this.m_driveSimulation.getModules()[0]),
                        new ModuleIOTalonFXSim(
                            TunerConstants.FrontRight, this.m_driveSimulation.getModules()[1]),
                        new ModuleIOTalonFXSim(
                            TunerConstants.BackLeft, this.m_driveSimulation.getModules()[2]),
                        new ModuleIOTalonFXSim(
                            TunerConstants.BackRight, this.m_driveSimulation.getModules()[3]),
                        m_driveSimulation::setSimulationWorldPose);


                m_profiledArm = new Arm(new ArmIOSim(), true);
                m_profiledElevator = new Elevator(new ElevatorIOSim(), true);
                // m_profiledClimber = new Climber(new ClimberIOSim(), true);
                m_clawRoller = new ClawRoller(new ClawRollerIOSim(), true);
                m_ClawRollerDS = new ClawRollerDS(new ClawRollerDSIOSim());
                m_IntakeDS = new IntakeDS(new IntakeDSIOSim());

                m_vision = new Vision(m_drive);

                break;

            default:
                // Replayed robot, disable IO implementations
                m_drive =
                    new Drive(
                        new GyroIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        new ModuleIO() {},
                        (robotPose) -> {
                        });

                m_profiledArm = new Arm(new ArmIO() {}, true);
                m_profiledElevator = new Elevator(new ElevatorIO() {}, true);
                // m_profiledClimber = new Climber(new ClimberIO() {}, true);
                m_clawRoller = new ClawRoller(new ClawRollerIO() {}, true);
                m_ClawRollerDS = new ClawRollerDS(new ClawRollerDSIO() {});
                m_IntakeDS = new IntakeDS(new IntakeDSIO() {});

                m_vision = new Vision(m_drive);
                break;
        }

        // Superstructure coordinates Arm and Elevator motions
        m_superStruct = new Superstructure(m_profiledArm, m_profiledElevator);

        // Logic Triggers
        registerNamedCommands();

        // Set up auto routines
        m_autoChooser =
            new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        // Set up SysId routines
        m_autoChooser.addOption(
            "Drive Wheel Radius Characterization",
            DriveCommands.wheelRadiusCharacterization(m_drive));
        m_autoChooser.addOption(
            "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(m_drive));
        m_autoChooser.addOption(
            "Drive SysId (Quasistatic Forward)",
            m_drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        m_autoChooser.addOption(
            "Drive SysId (Quasistatic Reverse)",
            m_drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        m_autoChooser.addOption(
            "Drive SysId (Dynamic Forward)", m_drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        m_autoChooser.addOption(
            "Drive SysId (Dynamic Reverse)", m_drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        // Configure the controller button and joystick bindings
        configureControllerBindings();

        // Detect if controllers are missing / Stop multiple warnings
        DriverStation.silenceJoystickConnectionWarning(true);
    }

    private Command joystickDrive()
    {
        return DriveCommands.joystickDrive(
            m_drive,
            () -> m_driver.getLeftY() * speedMultiplier,
            () -> m_driver.getLeftX() * speedMultiplier,
            () -> -m_driver.getRightX());
    }

    private Command joystickDriveAtAngle(Supplier<Rotation2d> angle)
    {
        return DriveCommands.joystickDriveAtAngle(
            m_drive,
            () -> m_driver.getLeftY() * speedMultiplier,
            () -> m_driver.getLeftX() * speedMultiplier,
            angle);
    }

    private Command joystickApproach(Supplier<Pose2d> approachPose)
    {
        return DriveCommands.joystickApproach(
            m_drive,
            () -> m_driver.getLeftY() * speedMultiplier,
            approachPose);
    }

    public Command setCoralAlgaeModeCommand()
    {
        return Commands.runOnce(
            () -> {
                coralModeEnabled = !coralModeEnabled;
            });
    }

    /** Button and Command mappings */
    private void configureControllerBindings()
    {
        // Default command, normal field-relative drive
        m_drive.setDefaultCommand(joystickDrive());

        // Reset gyro to 0° when start button is pressed
        final Runnable resetGyro =
            Constants.currentMode == Constants.Mode.SIM
                ? () -> m_drive.setPose(
                    m_driveSimulation
                        .getSimulatedDriveTrainPose()) // reset odometry to actual robot pose during
                // simulation
                : () -> m_drive.setPose(
                    new Pose2d(m_drive.getPose().getTranslation(), new Rotation2d())); // zero gyro
        m_driver.start().onTrue(Commands.runOnce(resetGyro, m_drive).ignoringDisable(true));

        // Driver Right Bumper: Approach Nearest Right-Side Reef Branch
        m_driver.rightBumper().and(isCoralMode)
            .whileTrue(
                joystickApproach(
                    () -> FieldConstants.getNearestReefBranch(m_drive.getPose(), ReefSide.RIGHT)));

        // Driver Left Bumper: Approach Nearest Left-Side Reef Branch
        m_driver.leftBumper().and(isCoralMode)
            .whileTrue(
                joystickApproach(
                    () -> FieldConstants.getNearestReefBranch(m_drive.getPose(), ReefSide.LEFT)));

        // Driver Left Bumper and Algae mode: Approach Nearest Reef Face
        m_driver.rightBumper().and(isCoralMode.negate())
            .whileTrue(
                joystickApproach(() -> FieldConstants.getNearestReefFace(m_drive.getPose())));

        // Driver A Button: Send Arm and Elevator to LEVEL_1
        m_driver.a().or(m_operator.a())
            .and(isCoralMode)
            .onTrue(
                m_superStruct.getTransitionCommand(Arm.State.LEVEL_1, Elevator.State.LEVEL_1)
                    .andThen(m_driver.rumbleForTime(1, 1)));

        // Driver A Button held and Algae mode: Send Arm and Elevator to Processor
        m_driver.a().or(m_operator.a())
            .and(isCoralMode.negate())
            .onTrue(
                m_superStruct.getTransitionCommand(Arm.State.ALGAE_GROUND,
                    Elevator.State.ALGAE_SCORE)
                    .andThen(m_driver.rumbleForTime(1, 1)));

        // Driver X Button: Send Arm and Elevator to LEVEL_2
        m_driver.x().or(m_operator.x())
            .and(isCoralMode)
            .onTrue(
                m_superStruct.getTransitionCommand(Arm.State.LEVEL_2, Elevator.State.LEVEL_2)
                    .andThen(m_driver.rumbleForTime(1, 1)));

        // Driver X Button and Algae mode: Send Arm and Elevator to ALGAE_LOW position
        m_driver.x().or(m_operator.x())
            .and(isCoralMode.negate())
            .onTrue(Commands.parallel(
                m_superStruct.getTransitionCommand(Arm.State.ALGAE_LOW,
                    Elevator.State.ALGAE_LOW)
                    .andThen(m_driver.rumbleForTime(1, 1)),
                m_clawRoller.setStateCommand(ClawRoller.State.EJECT)
                    .finallyDo(() -> m_clawRoller.setStateCommand(ClawRoller.State.OFF))));

        // Driver B Button: Send Arm and Elevator to LEVEL_3
        m_driver.b().or(m_operator.b())
            .and(isCoralMode)
            .onTrue(
                m_superStruct.getTransitionCommand(Arm.State.LEVEL_3, Elevator.State.LEVEL_3)
                    .andThen(m_driver.rumbleForTime(1, 1)));

        // Driver B Button and Algae mode: Send Arm and Elevator to ALGAE_HIGH position
        m_driver.b().or(m_operator.b())
            .and(isCoralMode.negate())
            .onTrue(Commands.parallel(
                m_superStruct.getTransitionCommand(Arm.State.ALGAE_HIGH,
                    Elevator.State.ALGAE_HIGH)
                    .andThen(m_driver.rumbleForTime(1, 1)),
                m_clawRoller.setStateCommand(ClawRoller.State.EJECT)
                    .finallyDo(() -> m_clawRoller.setStateCommand(ClawRoller.State.OFF))));

        // Driver Y Button: Send Arm and Elevator to LEVEL_4
        m_driver.y().or(m_operator.y())
            .and(isCoralMode)
            .onTrue(
                m_superStruct.getTransitionCommand(Arm.State.LEVEL_4, Elevator.State.LEVEL_4, 0.0,
                    0.8)
                    .andThen(m_driver.rumbleForTime(1, 1)));

        // Driver Right Trigger: Place Coral or Algae (Should be done once the robot is in position)
        m_driver.rightTrigger().and(isCoralMode)
            .and(() -> !(m_profiledElevator.isL1() || m_profiledElevator.isL4()))
            .whileTrue(Commands.parallel(
                m_clawRoller.setStateCommand(ClawRoller.State.SCORE)
                    .andThen(m_driver.rumbleForTime(1, 1)),
                // backup after shoot
                DriveCommands.joystickDrive(m_drive, () -> 0.9, () -> 0, () -> 0)
                    .withTimeout(0.6)
                    .andThen(DriveCommands.joystickDrive(m_drive, () -> 0, () -> 0, () -> 0))))
            .onFalse(Commands.waitUntil(m_ClawRollerDS.triggered.negate())
                .andThen(m_clawRoller.setStateCommand(ClawRoller.State.OFF))
                .andThen(m_superStruct.getTransitionCommand(Arm.State.CORAL_INTAKE,
                    Elevator.State.STOW)));

        m_driver.rightTrigger().and(isCoralMode).and(() -> m_profiledElevator.isL1())
            .whileTrue(Commands.parallel(
                m_clawRoller.setStateCommand(ClawRoller.State.SCORE_L1),
                // backup after shoot
                DriveCommands.joystickDrive(m_drive, () -> 0.5, () -> 0, () -> 0)
                    .withTimeout(0.6)
                    .andThen(DriveCommands.joystickDrive(m_drive, () -> 0, () -> 0, () -> 0))))
            .onFalse(Commands.waitUntil(m_ClawRollerDS.triggered.negate())
                .andThen(m_clawRoller.setStateCommand(ClawRoller.State.OFF))
                // .andThen(Commands.waitSeconds(1))
                .andThen(m_superStruct.getTransitionCommand(Arm.State.CORAL_INTAKE,
                    Elevator.State.CORAL_INTAKE)));

        m_driver.rightTrigger().and(isCoralMode).and(() -> m_profiledElevator.isL4())
            .whileTrue(Commands.parallel(
                m_clawRoller.setStateCommand(ClawRoller.State.SCORE_L4),
                // backup after shoot
                DriveCommands.joystickDrive(m_drive, () -> 0.9, () -> 0, () -> 0)
                    .withTimeout(0.8)
                    .andThen(DriveCommands.joystickDrive(m_drive, () -> 0, () -> 0, () -> 0))))
            .onFalse(Commands.waitUntil(m_ClawRollerDS.triggered.negate())
                .andThen(m_clawRoller.setStateCommand(ClawRoller.State.OFF))
                // .andThen(Commands.waitSeconds(1))
                .andThen(m_superStruct.getTransitionCommand(Arm.State.CORAL_INTAKE,
                    Elevator.State.CORAL_INTAKE)));

        // Driver Left Trigger: Drivetrain drive at coral station angle, prepare the elevator and
        // arm, Get Ready to Intake Coral
        // m_driver
        // .leftTrigger().and(isCoralMode)
        // .whileTrue(
        // m_clawRoller.setStateCommand(ClawRoller.State.INTAKESLOW)
        // .andThen(
        // m_superStruct
        // .getTransitionCommand(Arm.State.CORAL_INTAKE,
        // Elevator.State.CORAL_INTAKE))
        // .andThen(
        // Commands.waitUntil(m_IntakeDS.triggered
        // .and(m_ClawRollerDS.triggered.negate())))
        // .andThen(
        // Commands.waitUntil(m_IntakeDS.triggered.negate()
        // .and(m_ClawRollerDS.triggered)))
        // .andThen(m_clawRoller.holdCoralCommand(m_ClawRollerDS.triggered)));

        m_driver
            .leftTrigger().and(isCoralMode)
            .whileTrue(
                m_clawRoller.setStateCommand(ClawRoller.State.INTAKE)
                    .andThen(m_superStruct.getTransitionCommand(Arm.State.CORAL_INTAKE,
                        Elevator.State.CORAL_INTAKE))
                    .andThen(m_driver.rumbleForTime(1, 1))
                    .andThen(Commands.waitUntil(m_ClawRollerDS.triggered))
                    .andThen(Commands.waitSeconds(0.25))
                    .andThen(m_clawRoller.setStateCommand(ClawRoller.State.HOLDCORAL)))
            .onFalse(
                Commands.either(
                    m_clawRoller.setStateCommand(ClawRoller.State.OFF),
                    Commands
                        .waitUntil(m_IntakeDS.triggered.negate()
                            .and(m_ClawRollerDS.triggered))
                        .andThen(m_clawRoller.setStateCommand(ClawRoller.State.HOLDCORAL)),
                    m_IntakeDS.triggered
                        .and(m_ClawRollerDS.triggered).negate()));

        // Driver Start Button: Climb Request (toggle)
        // m_driver.start().onTrue(Commands.runOnce(() -> {
        // m_profiledClimber.climbRequested = true;
        // m_profiledClimber.climbStep += 1;
        // }));

        // Climb step 1: Get the Arm Down, then the Elevator down, and then and move climber to prep
        // m_profiledClimber.getClimbRequest().and(m_profiledClimber.getClimbStep1()).whileTrue(
        // Commands.parallel(
        // Commands.parallel(
        // m_profiledArm.setStateCommand(Arm.State.CLIMB),
        // Commands.waitUntil(() -> m_profiledArm.atPosition(0.1))
        // .andThen(m_profiledElevator.setStateCommand(Elevator.State.STOW))),
        // Commands
        // .waitUntil(
        // () -> m_profiledElevator.atPosition(0.1) && m_profiledArm.atPosition(0.1)))
        // .andThen(m_profiledClimber.setStateCommand(Climber.State.PREP)));

        // Climb step 2: Move climber to climb
        // m_profiledClimber.getClimbRequest().and(m_profiledClimber.getClimbStep2())
        // .whileTrue(
        // m_profiledClimber.setStateCommand(Climber.State.CLIMB)
        // .until(m_profiledClimber.getClimbedTrigger()));

        // m_profiledClimber.getClimbedTrigger().onTrue(m_profiledClimber.climbedAlertCommand());

        // Driver POV Right: End Climbing Sequence if needed
        // m_driver
        // .povRight()
        // .onTrue(
        // Commands.runOnce(
        // () -> {
        // m_profiledClimber.climbRequested = false;
        // m_profiledClimber.climbStep = 0;
        // }));

        // Slow drivetrain to 25% while climbing
        // m_profiledClimber.getClimbRequest().whileTrue(
        // DriveCommands.joystickDrive(
        // m_drive,
        // () -> -m_driver.getLeftY() * 0.25,
        // () -> -m_driver.getLeftX() * 0.25,
        // () -> -m_driver.getRightX() * 0.25));

        // Driver POV Down: Zero the Elevator (HOMING)
        m_driver.povDown().whileTrue(
            Commands.sequence(
                // Always move Arm to STOW position before moving Elevator
                m_profiledArm.setStateCommand(Arm.State.STOW),
                // Move Elevator to homing position
                Commands.waitUntil(() -> m_profiledArm.atPosition(0.1)),
                m_profiledElevator.setStateCommand(Elevator.State.HOMING),
                Commands.waitUntil(m_profiledElevator.getHomedTrigger()),
                m_profiledElevator.zeroSensorCommand(),
                m_profiledElevator.setStateCommand(Elevator.State.STOW)));

        m_driver.povUp().onTrue(
            m_profiledElevator.setStateCommand(Elevator.State.STOW));

        // Driver Right Bumper: Toggle between Coral and Algae Modes.
        // Make sure the Approach nearest reef face does not mess with this
        m_driver.back().and(m_driver.leftBumper().negate())
            .onTrue(setCoralAlgaeModeCommand()
                .andThen(m_superStruct.getTransitionCommand(Arm.State.STOW, Elevator.State.STOW))
                .andThen(m_clawRoller.setStateCommand(ClawRoller.State.OFF)));

    }

    /**
     * Register Named commands for use in PathPlanner
     */
    private void registerNamedCommands()
    {
        // Go to the L1 Position
        NamedCommands.registerCommand(
            "L1",
            Commands.waitUntil(m_ClawRollerDS.triggered).andThen(
                m_superStruct.getTransitionCommand(Arm.State.LEVEL_1, Elevator.State.LEVEL_1, 0.1,
                    0.8)));
        // Go to the L2 Position
        NamedCommands.registerCommand(
            "L2Prep",
            Commands.waitUntil(m_ClawRollerDS.triggered).andThen(
                m_superStruct.getTransitionCommand(Arm.State.STOW, Elevator.State.LEVEL_2, 0.1,
                    0.8)));
        // Go to the L3 Position
        NamedCommands.registerCommand(
            "L3Prep",
            Commands.waitUntil(m_ClawRollerDS.triggered).andThen(
                m_superStruct.getTransitionCommand(Arm.State.STOW, Elevator.State.LEVEL_3, 0.1,
                    0.8)));
        // Go to the L4 Position
        NamedCommands.registerCommand(
            "L4",
            Commands.waitUntil(m_ClawRollerDS.triggered)
                .andThen(
                    m_superStruct.getTransitionCommand(Arm.State.LEVEL_4, Elevator.State.LEVEL_4,
                        0.08,
                        0.8)));

        NamedCommands.registerCommand(
            "L4Prep",
            Commands.waitUntil(m_ClawRollerDS.triggered)
                .andThen(
                    m_superStruct.getTransitionCommand(Arm.State.STOW, Elevator.State.LEVEL_4,
                        0.1,
                        0.8)));
        // Go to the Home Position
        NamedCommands.registerCommand(
            "Home",
            m_superStruct.getTransitionCommand(Arm.State.STOW, Elevator.State.STOW, 0.1, 0.8));

        // Wait for EE sensor to be triggered
        NamedCommands.registerCommand("SuperstructureIntake",
            m_superStruct
                .getTransitionCommand(Arm.State.CORAL_INTAKE,
                    Elevator.State.CORAL_INTAKE, 0.1, 0.8)
                .andThen(m_clawRoller.setStateCommand(ClawRoller.State.INTAKE)));

        NamedCommands.registerCommand(
            "WaitForCoral",
            Commands.waitUntil(m_IntakeDS.triggered));

        // Intake Coral
        NamedCommands.registerCommand(
            "IntakeCoral",
            m_clawRoller.setStateCommand(ClawRoller.State.INTAKE)
                .andThen(
                    Commands.waitUntil(
                        m_IntakeDS.triggered.negate().and(m_ClawRollerDS.triggered)))
                .andThen(m_clawRoller.setStateCommand(ClawRoller.State.HOLDCORAL)));


        NamedCommands.registerCommand(
            "Score",
            m_clawRoller.setStateCommand(ClawRoller.State.SCORE)
                .andThen(Commands.waitUntil(m_ClawRollerDS.triggered.negate()))
                .andThen(m_clawRoller.setStateCommand(ClawRoller.State.OFF)));

        NamedCommands.registerCommand("Coast", m_drive.run(() -> {
        }));

        NamedCommands.registerCommand("HoldCoral",
            m_clawRoller.setStateCommand(ClawRoller.State.HOLDCORAL));

        NamedCommands.registerCommand("WaitForEnd", Commands.waitSeconds(14.7));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand()
    {
        return m_autoChooser.get();
    }

    /*
     * Simulation-specific routines
     */
    public void resetSimulation()
    {
        if (Constants.currentMode != Constants.Mode.SIM)
            return;

        m_drive.setPose(new Pose2d(3, 3, new Rotation2d()));
        SimulatedArena.getInstance().resetFieldForAuto();
    }

    public void resetSimulationField()
    {
        if (Constants.currentMode != Constants.Mode.SIM)
            return;
    }

    public void displaySimFieldToAdvantageScope()
    {
        if (Constants.currentMode != Constants.Mode.SIM)
            return;

        // SimulatedArena.getInstance().addGamePiece(new ReefscapeAlgaeOnField(new Translation2d(2,
        // 2)));
        Logger.recordOutput(
            "FieldSimulation/RobotPosition", m_driveSimulation.getSimulatedDriveTrainPose());
        Logger.recordOutput(
            "FieldSimulation/Coral",
            SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
        Logger.recordOutput(
            "FieldSimulation/Algae",
            SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
    }
}
