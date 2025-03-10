// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
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
import frc.robot.subsystems.Climber.ClimberIOTalonFX;
import frc.robot.subsystems.Elevator.*;
import frc.robot.subsystems.Vision.*;
import frc.robot.subsystems.drive.*;
import frc.robot.util.WindupXboxController;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
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
    private final WindupXboxController m_operator = new WindupXboxController(1);

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> m_autoChooser;

    // Maple Sim
    private SwerveDriveSimulation m_driveSimulation = null;

    // AK-enabled Subsystems
    public final Drive m_drive;
    private final Arm m_profiledArm;
    private final Elevator m_profiledElevator;
    private final Climber m_profiledClimber;
    private final ClawRoller m_clawRoller;
    private final ClawRollerDS m_ClawRollerDS;
    public final IntakeDS m_IntakeDS;
    private final Superstructure m_superStruct;

    public final Vision m_vision;

    @RequiredArgsConstructor
    @Getter
    public enum RobotMode {
        CORAL,
        ALGAE,
        GAP;
    }

    @Getter
    @Setter
    private RobotMode robotMode = RobotMode.CORAL;

    // Triggers for mode switching
    private Trigger isCoralMode = new Trigger(() -> RobotMode.CORAL == robotMode);
    private Trigger isGapMode = new Trigger(() -> RobotMode.GAP == robotMode);
    private Trigger isAlgaeMode = new Trigger(() -> RobotMode.ALGAE == robotMode);

    private double speedMultiplier = 0.85;
    private Supplier<Double> speedScalar = () -> speedMultiplier;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer()
    {
        switch (Constants.currentMode) {
            case REAL:
                m_vision = new Vision(new VisionIOQuestNav(VisionConstants.questName));

                // Real robot, instantiate hardware IO implementations
                m_drive =
                    new Drive(
                        new GyroIOPigeon2(),
                        new ModuleIOTalonFXReal(TunerConstants.FrontLeft),
                        new ModuleIOTalonFXReal(TunerConstants.FrontRight),
                        new ModuleIOTalonFXReal(TunerConstants.BackLeft),
                        new ModuleIOTalonFXReal(TunerConstants.BackRight),
                        m_vision::zeroQuest);

                m_vision.updateConsumer(m_drive);
                m_profiledArm = new Arm(new ArmIOTalonFX(), false);
                m_profiledElevator = new Elevator(new ElevatorIOTalonFX(), false);
                m_profiledClimber = new Climber(new ClimberIOTalonFX(), false);
                m_clawRoller = new ClawRoller(new ClawRollerIOTalonFX(), false);
                m_ClawRollerDS = new ClawRollerDS(new ClawRollerDSIOReal());
                m_IntakeDS = new IntakeDS(new IntakeDSIOReal());

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
                m_profiledClimber = new Climber(new ClimberIOSim(), true);
                m_clawRoller = new ClawRoller(new ClawRollerIOSim(), true);
                m_ClawRollerDS = new ClawRollerDS(new ClawRollerDSIOSim());
                m_IntakeDS = new IntakeDS(new IntakeDSIOSim());

                m_vision = new Vision();
                m_vision.updateConsumer(m_drive);

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
                m_profiledClimber = new Climber(new ClimberIO() {}, true);
                m_clawRoller = new ClawRoller(new ClawRollerIO() {}, true);
                m_ClawRollerDS = new ClawRollerDS(new ClawRollerDSIO() {});
                m_IntakeDS = new IntakeDS(new IntakeDSIO() {});

                m_vision = new Vision();
                m_vision.updateConsumer(m_drive);
                break;
        }

        // Superstructure coordinates Arm and Elevator motions
        m_superStruct = new Superstructure(m_profiledArm, m_profiledElevator);

        // Logic Triggers
        registerNamedCommands();

        // Set up auto routines
        m_autoChooser =
            new LoggedDashboardChooser<>("Auto Choices",
                AutoBuilder.buildAutoChooser("3 Piece Right"));

        for (String auto : AutoBuilder.getAllAutoNames()) {
            if (auto.contains("Right")) {
                m_autoChooser.addOption(auto.replace("Right", "Left"),
                    new PathPlannerAuto(auto, true));
            }
        }

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
            () -> -m_driver.getLeftY() * speedScalar.get(),
            () -> -m_driver.getLeftX() * speedScalar.get(),
            () -> -m_driver.getRightX() * 0.75);
    }

    private Command joystickDriveAtAngle(Supplier<Rotation2d> angle)
    {
        return DriveCommands.joystickDriveAtAngle(
            m_drive,
            () -> -m_driver.getLeftY() * speedMultiplier,
            () -> -m_driver.getLeftX() * speedMultiplier,
            angle);
    }

    private Command joystickApproach(Supplier<Pose2d> approachPose)
    {
        return DriveCommands.joystickApproach(
            m_drive,
            () -> -m_driver.getLeftY() * speedMultiplier,
            approachPose);
    }

    public Command setRobotModeCommand(RobotMode mode)
    {
        return Commands.runOnce(
            () -> {
                robotMode = mode;
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
            .whileTrue(Commands.either(
                joystickApproach(
                    () -> FieldConstants.getNearestReefBranch(m_drive.getPose(), ReefSide.RIGHT)),
                joystickDriveAtAngle(
                    () -> FieldConstants.getNearestCoralStation(m_drive.getPose()).getRotation()),
                m_ClawRollerDS.triggered));

        // Driver Left Bumper: Approach Nearest Left-Side Reef Branch
        m_driver.leftBumper().and(isCoralMode)
            .whileTrue(Commands.either(
                joystickApproach(
                    () -> FieldConstants.getNearestReefBranch(m_drive.getPose(), ReefSide.LEFT)),
                joystickDriveAtAngle(
                    () -> FieldConstants.getNearestCoralStation(m_drive.getPose()).getRotation()),
                m_ClawRollerDS.triggered));

        // Driver Right Bumper and Algae mode: Approach Nearest Reef Face
        m_driver.rightBumper().and(isAlgaeMode)
            .whileTrue(
                joystickApproach(() -> FieldConstants.getNearestReefFace(m_drive.getPose())));

        // Driver A Button: Send Arm and Elevator to LEVEL_1
        m_driver.a().or(m_operator.a())
            .and(isCoralMode).and(() -> m_clawRoller.notIntaking())
            .onTrue(
                Commands.runOnce(() -> speedMultiplier = 0.85).andThen(
                    m_superStruct.getTransitionCommand(Arm.State.LEVEL_1, Elevator.State.LEVEL_1)));

        // Driver X Button: Send Arm and Elevator to LEVEL_2
        m_driver.x().or(m_operator.x())
            .and(isCoralMode).and(() -> m_clawRoller.notIntaking())
            .onTrue(
                Commands.runOnce(() -> speedMultiplier = 0.85).andThen(
                    m_superStruct.getTransitionCommand(Arm.State.LEVEL_2, Elevator.State.LEVEL_2)));

        // Driver X Button and Algae mode: Send Arm and Elevator to ALGAE_LOW position
        m_driver.x().or(m_operator.x())
            .and(isAlgaeMode)
            .onTrue(Commands.parallel(
                m_superStruct.getTransitionCommand(Arm.State.ALGAE_LOW,
                    Elevator.State.ALGAE_LOW),
                m_clawRoller.setStateCommand(ClawRoller.State.EJECT)));

        // Driver B Button: Send Arm and Elevator to LEVEL_3
        m_driver.b().or(m_operator.b())
            .and(isCoralMode).and(() -> m_clawRoller.notIntaking())
            .onTrue(
                Commands.runOnce(() -> speedMultiplier = 0.5).andThen(
                    m_superStruct.getTransitionCommand(Arm.State.LEVEL_3, Elevator.State.LEVEL_3)));

        // Driver B Button and Algae mode: Send Arm and Elevator to ALGAE_HIGH position
        m_driver.b().or(m_operator.b())
            .and(isAlgaeMode)
            .onTrue(Commands.parallel(
                m_superStruct.getTransitionCommand(Arm.State.ALGAE_HIGH,
                    Elevator.State.ALGAE_HIGH),
                m_clawRoller.setStateCommand(ClawRoller.State.EJECT)));

        // Driver Y Button: Send Arm and Elevator to LEVEL_4
        m_driver.y().or(m_operator.y())
            .and(isCoralMode).and(() -> m_clawRoller.notIntaking())
            .onTrue(
                Commands.runOnce(() -> speedMultiplier = 0.5).andThen(
                    m_superStruct.getTransitionCommand(Arm.State.LEVEL_4, Elevator.State.LEVEL_4)));

        // Driver Right Trigger: Any other level
        m_driver.rightTrigger().and(isCoralMode).and(() -> m_profiledElevator.isOther())
            .onTrue(m_clawRoller.setStateCommand(ClawRoller.State.HOLDCORAL));

        // Driver Right Trigger: Place Coral on L2/L3
        m_driver.rightTrigger().and(isCoralMode)
            .and(() -> m_profiledElevator.isL2() || m_profiledElevator.isL3())
            .onTrue(Commands.parallel(
                m_clawRoller.setStateCommand(ClawRoller.State.SCORE)
                    .andThen(Commands.waitUntil(m_ClawRollerDS.triggered.negate()))
                    .andThen(m_clawRoller.setStateCommand(ClawRoller.State.HOLDCORAL)),
                // backup after shoot
                DriveCommands.joystickDriveRobot(m_drive, () -> -0.15, () -> 0, () -> 0)
                    .withTimeout(0.4)
                    .andThen(Commands.runOnce(() -> m_drive.stop()))
                    .andThen(m_superStruct.getTransitionCommand(Arm.State.CORAL_INTAKE,
                        Elevator.State.CORAL_INTAKE))));

        // Place Coral on L1
        m_driver.rightTrigger().and(isCoralMode).and(() -> m_profiledElevator.isL1())
            .onTrue(
                m_clawRoller.setStateCommand(ClawRoller.State.SCORE_L1)
                    .andThen(m_profiledArm.setStateCommand(Arm.State.LEVEL_1_FLIP))
                    .andThen(Commands.waitUntil(m_ClawRollerDS.triggered.negate()))
                    .andThen(m_profiledElevator.setStateCommand(Elevator.State.LEVEL_1_GAP))
                    .andThen(m_clawRoller.setStateCommand(ClawRoller.State.HOLDCORAL)));

        // Place Coral on L4
        m_driver.rightTrigger().and(isCoralMode).and(() -> m_profiledElevator.isL4())
            .onTrue(
                m_clawRoller.setStateCommand(ClawRoller.State.SCORE_L4)
                    .andThen(m_profiledArm.setStateCommand(Arm.State.LEVEL_4_BACK))
                    .andThen(Commands.waitUntil(m_ClawRollerDS.triggered.negate())
                        .andThen(m_clawRoller.setStateCommand(ClawRoller.State.HOLDCORAL))
                        .withTimeout(1))
                    // backup after shoot
                    .andThen(
                        DriveCommands.joystickDriveRobot(m_drive, () -> -0.15, () -> 0, () -> 0)
                            .withTimeout(0.5))
                    .andThen(Commands.runOnce(() -> m_drive.stop()))
                    .andThen(Commands.runOnce(() -> speedMultiplier = 1.0))
                    .andThen(m_superStruct.getTransitionCommand(Arm.State.CORAL_INTAKE,
                        Elevator.State.CORAL_INTAKE)));

        // Intake coral from chute if you don't already have one
        m_driver.leftTrigger().or(m_operator.leftTrigger()).and(isCoralMode)
            .and(m_ClawRollerDS.triggered.negate())
            .onTrue(
                m_clawRoller.setStateCommand(ClawRoller.State.INTAKE)
                    .andThen(m_superStruct.getTransitionCommand(Arm.State.CORAL_INTAKE,
                        Elevator.State.CORAL_INTAKE))
                    .andThen(Commands.waitUntil(m_ClawRollerDS.triggered))
                    .andThen(Commands.waitSeconds(0.25))
                    .andThen(m_clawRoller.setStateCommand(ClawRoller.State.HOLDCORAL)));

        // Driver or Operator POV Down: Zero the Elevator (HOMING)
        m_driver.povDown().or(m_operator.povDown()).whileTrue(
            Commands.sequence(
                // Always move Arm to STOW position before moving Elevator
                m_profiledArm.setStateCommand(Arm.State.STOW),
                // Move Elevator to homing position
                Commands.waitUntil(() -> m_profiledArm.atPosition(0.1)),
                m_profiledElevator.setStateCommand(Elevator.State.HOMING),
                Commands.waitUntil(m_profiledElevator.getHomedTrigger()),
                m_profiledElevator.zeroSensorCommand(),
                m_profiledElevator.setStateCommand(Elevator.State.CORAL_INTAKE)));

        // Driver Back: Zero the Arm (HOMING)
        m_driver.back().whileTrue(
            Commands.sequence(
                // Move Arm to CORAL_INTAKE position before homing
                m_profiledArm.setStateCommand(Arm.State.CORAL_INTAKE),
                Commands.waitUntil(() -> m_profiledArm.atPosition(0.1)),
                m_profiledArm.setStateCommand(Arm.State.HOMING),
                Commands.waitUntil(m_profiledArm.getHomedTrigger()),
                m_profiledArm.zeroSensorCommand(),
                m_profiledArm.setStateCommand(Arm.State.CORAL_INTAKE)));

        // Left DPAD sets Coral Mode
        m_driver.povLeft().or(m_operator.povLeft())
            .onTrue(setRobotModeCommand(RobotMode.CORAL));

        // Up DPAD sets Algae Mode
        m_driver.povUp().or(m_operator.povUp())
            .onTrue(setRobotModeCommand(RobotMode.ALGAE));

        // Right DPAD sets Gap Mode
        m_driver.povRight().or(m_operator.povRight())
            .onTrue(setRobotModeCommand(RobotMode.GAP));

        // While operator start held winch in the climber
        m_operator.start().onTrue(m_profiledArm.setStateCommand(Arm.State.CLIMB));
        m_operator.start().whileTrue(m_profiledClimber.setStateCommand(Climber.State.CLIMB))
            .onFalse(m_profiledClimber.setStateCommand(Climber.State.HOME));

        // While operator back held release the climber
        m_operator.back().whileTrue(m_profiledClimber.setStateCommand(Climber.State.UNCLIMB))
            .onFalse(m_profiledClimber.setStateCommand(Climber.State.HOME));

        // For custom tuning in AdvantageScope
        m_operator.rightTrigger().onTrue(m_profiledElevator.setStateCommand(Elevator.State.TUNING));
        m_operator.leftBumper().onTrue(m_profiledArm.setStateCommand(Arm.State.TUNING));
    }

    /**
     * Register Named commands for use in PathPlanner
     */
    private void registerNamedCommands()
    {
        // Go to the L4 Position
        NamedCommands.registerCommand("L4",
            Commands.waitUntil(m_ClawRollerDS.triggered)
                .andThen(
                    m_superStruct.getTransitionCommand(Arm.State.LEVEL_4, Elevator.State.LEVEL_4,
                        0.1,
                        0.1)));

        // Go to the Home Position
        NamedCommands.registerCommand("Home",
            m_superStruct.getTransitionCommand(Arm.State.CORAL_INTAKE, Elevator.State.CORAL_INTAKE,
                0.1,
                0.1));

        // Move Elevator and Arm then wait for EE sensor to be triggered
        NamedCommands.registerCommand("Intake",
            m_clawRoller.setStateCommand(ClawRoller.State.INTAKE)
                .andThen(Commands.waitUntil(m_ClawRollerDS.triggered))
                .andThen(Commands.waitSeconds(0.25))
                .andThen(m_clawRoller.setStateCommand(ClawRoller.State.HOLDCORAL)));

        // Score Coral on L4 and shut off when it leaves
        NamedCommands.registerCommand("Shoot",
            m_clawRoller.setStateCommand(ClawRoller.State.SCORE_L4)
                .andThen(m_profiledArm.setStateCommand(Arm.State.LEVEL_4_BACK)));
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
