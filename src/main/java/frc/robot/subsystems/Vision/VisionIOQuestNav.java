package frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.FloatArraySubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VisionIOQuestNav implements VisionIO {
    // Configure Network Tables topics (questnav/...) to communicate with the Quest HMD
    NetworkTableInstance nt4Instance = NetworkTableInstance.getDefault();
    NetworkTable nt4Table = nt4Instance.getTable("questnav");
    private IntegerSubscriber questMiso = nt4Table.getIntegerTopic("miso").subscribe(0);
    private IntegerPublisher questMosi = nt4Table.getIntegerTopic("mosi").publish();
    public Field2d fieldMap = new Field2d();

    // Subscribe to the Network Tables questnav data topics
    private DoubleSubscriber questTimestamp = nt4Table.getDoubleTopic("timestamp").subscribe(0.0f);
    private FloatArraySubscriber questPosition =
        nt4Table.getFloatArrayTopic("position").subscribe(new float[] {0.0f, 0.0f, 0.0f});
    private FloatArraySubscriber questQuaternion =
        nt4Table.getFloatArrayTopic("quaternion").subscribe(new float[] {0.0f, 0.0f, 0.0f, 0.0f});
    private FloatArraySubscriber questEulerAngles =
        nt4Table.getFloatArrayTopic("eulerAngles").subscribe(new float[] {0.0f, 0.0f, 0.0f});
    private DoubleSubscriber questBatteryPercent =
        nt4Table.getDoubleTopic("batteryPercent").subscribe(0.0f);

    // Pose of the Quest when the pose was reset
    private Pose2d resetPoseOculus = new Pose2d();

    // Pose of the robot when the pose was reset
    private Pose2d resetPoseRobot = new Pose2d();

    // Position of the quest on the robot (6" forward, centered side-to-side, pointed forward))
    private final Transform2d robotToQuest =
        new Transform2d(Units.inchesToMeters(-7.5), Units.inchesToMeters(-11),
            new Rotation2d(Units.degreesToRadians(-120)));

    public VisionIOQuestNav(String name)
    {
        // Zero the absolute 3D position of the robot (similar to long-pressing the quest logo)
        if (questMiso.get() != 99) {
            questMosi.set(1);
        }
        SmartDashboard.putData("Quest Field Map", fieldMap);
    }

    @Override
    public void updateInputs(VisionIOInputs inputs)
    {
        inputs.connected = isConnected();
        var pose = getRobotPose();

        // Read new pose observations from NetworkTables
        inputs.tagIds = new int[0];
        inputs.poseObservations =
            new VisionIO.PoseObservation[] {
                    new PoseObservation(
                        getTimestamp(), // Timestamp
                        new Pose3d(pose), // 3D pose estimate
                        0, // Ambiguity
                        11, // Tag count
                        0.0) // Average tag distance
            }; // Observation type

        fieldMap.setRobotPose(pose);

        cleanUpQuestNavMessages();
    }

    /**
     * Gets the pose of the robot on the field
     *
     * @return pose of the robot
     */
    public Pose2d getRobotPose()
    {
        return getQuestPose().transformBy(robotToQuest.inverse());
    }

    /**
     * Gets the pose of the Quest on the field
     *
     * @return pose of the Quest
     */
    public Pose2d getQuestPose()
    {
        var rawPose = getUncorrectedOculusPose();
        var poseRelativeToReset = rawPose.minus(resetPoseOculus);

        return resetPoseRobot.transformBy(poseRelativeToReset);
    }

    /*
     * Gets the battery percent of the Quest.
     *
     * @return battery percent of the Quest
     */
    public double getBatteryPercent()
    {
        return questBatteryPercent.get();
    }

    /**
     * Returns if the Quest is connected
     *
     * @return true if the Quest is connected
     */
    public boolean isConnected()
    {
        return ((RobotController.getFPGATime() - questBatteryPercent.getLastChange()) / 1000) < 250;
    }

    /**
     * Gets the raw Rotation3d of the Quest
     *
     * @return Rotation3d of the Quest, not adjusted for the reset pose
     */
    public Rotation3d getQuaternion()
    {
        float[] qqFloats = questQuaternion.get();
        return new Rotation3d(new Quaternion(qqFloats[0], qqFloats[1], qqFloats[2], qqFloats[3]));
    }

    /**
     * Gets the Quests's timestamp
     *
     * @return the Quest timestamp
     */
    public double getTimestamp()
    {
        return questTimestamp.get();
    }

    /**
     * Set the robot's pose on the field. This is useful to seed the robot to a known position. This
     * is usually called at the start of the autonomous period.
     *
     * @param newPose new robot pose
     */
    @Override
    public void zeroQuest(Pose2d newPose)
    {
        resetPoseOculus = getUncorrectedOculusPose().transformBy(robotToQuest.inverse());
        resetPoseRobot = newPose;
    }

    /**
     * Clean up questnav subroutine messages after processing on the headset. Call this each
     * iteration to reset the command after it has been processed.
     */
    public void cleanUpQuestNavMessages()
    {
        if (questMiso.get() == 99) {
            questMosi.set(0);
        }
    }

    /**
     * Gets the raw pose of the oculus, relative to the position where it started
     *
     * @return pose of the oculus
     */
    private Pose2d getUncorrectedOculusPose()
    {
        var eulerAngles = questEulerAngles.get();
        var rotation = Rotation2d.fromDegrees(-Math.IEEEremainder(eulerAngles[1], 360d));

        var questnavPosition = questPosition.get();
        var translation = new Translation2d(questnavPosition[2], -questnavPosition[0]);
        return new Pose2d(translation, rotation);
    }
}
