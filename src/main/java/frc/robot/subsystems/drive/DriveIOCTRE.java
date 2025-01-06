// Copyright (c) 2024 FRC 5712
// Open Source Software, you can modify it according to the terms
// of the MIT License at the root of this project

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

/**
 * CTRE Phoenix6-based implementation of the DriveIO interface. Extends the Phoenix SwerveDrivetrain
 * class to provide swerve drive functionality with additional telemetry and simulation support.
 *
 * <p>This class handles: - Odometry data collection and buffering - Vision measurement integration
 * - Simulation state updates - Thread-safe telemetry updates
 */
public class DriveIOCTRE extends SwerveDrivetrain implements DriveIO {
  // Simulation constants
  private static final double SIMULATION_LOOP_PERIOD = 0.005; // 5 ms
  private Notifier simulationNotifier;
  private double lastSimulationTime;

  // Queue configuration
  private static final int QUEUE_SIZE = 20;

  // Thread-safe storage
  private final Lock odometryLock = new ReentrantLock();
  private final Queue<Rotation2d> gyroYawQueue = new ArrayBlockingQueue<>(QUEUE_SIZE);
  private final Queue<Double> timestampQueue = new ArrayBlockingQueue<>(QUEUE_SIZE);
  private final List<Queue<Double>> drivePositionQueues = new ArrayList<>();
  private final List<Queue<Rotation2d>> steerPositionQueues = new ArrayList<>();

  /**
   * Creates a new DriveIOCTRE with specified update frequency.
   *
   * @param driveTrainConstants Constants defining drivetrain behavior
   * @param odometryUpdateFrequency How often to update odometry in Hz
   * @param modules Array of constants for each swerve module
   */
  public DriveIOCTRE(
      SwerveDrivetrainConstants driveTrainConstants,
      double odometryUpdateFrequency,
      SwerveModuleConstants... modules) {
    super(driveTrainConstants, odometryUpdateFrequency, modules);
    setup();
  }

  /**
   * Creates a new DriveIOCTRE with default update frequency.
   *
   * @param driveTrainConstants Constants defining drivetrain behavior
   * @param modules Array of constants for each swerve module
   */
  public DriveIOCTRE(
      SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
    super(driveTrainConstants, modules);
    setup();
  }

  public DriveIOCTRE(
      SwerveDrivetrainConstants drivetrainConstants,
      double odometryUpdateFrequency,
      Matrix<N3, N1> odometryStandardDeviation,
      Matrix<N3, N1> visionStandardDeviation,
      SwerveModuleConstants... modules) {
    super(
        drivetrainConstants,
        odometryUpdateFrequency,
        odometryStandardDeviation,
        visionStandardDeviation,
        modules);
    setup();
  }

  /** Sets up the DriveIOCTRE with telemetry and simulation support if needed. */
  private void setup() {
    initializeQueues();
    // This pulls data from our Odometry thread or in this case at 250 Hz
    registerTelemetry(this::updateTelemetry);
    setupSimulation();
  }

  /** Initializes the position queues for drive and steer data. */
  private void initializeQueues() {
    for (int i = 0; i < 4; i++) {
      drivePositionQueues.add(new ArrayBlockingQueue<>(QUEUE_SIZE));
      steerPositionQueues.add(new ArrayBlockingQueue<>(QUEUE_SIZE));
    }
  }

  /** Sets up simulation if needed. */
  private void setupSimulation() {
    if (Constants.currentMode == Mode.SIM) {
      startSimThread();
    }
  }

  @Override
  public void updateInputs(DriveIOInputs inputs) {
    // Update state-based inputs
    SwerveDriveState state = getState();
    inputs.moduleStates = state.ModuleStates;
    inputs.moduleTargets = state.ModuleTargets;
    inputs.modulePositions = state.ModulePositions;
    inputs.pose = state.Pose;
    inputs.speeds = state.Speeds;
    inputs.odometryPeriod = state.OdometryPeriod;
    inputs.successfulDaqs = state.SuccessfulDaqs;
    inputs.failedDaqs = state.FailedDaqs;

    // Update sensor inputs
    inputs.gyroRate = getPigeon2().getAngularVelocityZWorld().getValue();
    inputs.gyroConnected = super.getPigeon2().isConnected();
    inputs.operatorForwardDirection = getOperatorForwardDirection();
    inputs.odometryIsValid = isOdometryValid();

    // Update queued data with thread safety
    odometryLock.lock();
    try {
      // Process timestamps
      inputs.timestamp = timestampQueue.stream().mapToDouble(Double::valueOf).toArray();
      inputs.gyroYaw = gyroYawQueue.stream().toArray(Rotation2d[]::new);

      for (int i = 0; i < getModules().length; i++) {
        inputs.drivePositions[i] =
            drivePositionQueues.get(i).stream().mapToDouble(Double::valueOf).toArray();
        inputs.steerPositions[i] = steerPositionQueues.get(i).stream().toArray(Rotation2d[]::new);

        drivePositionQueues.get(i).clear();
        steerPositionQueues.get(i).clear();
      }

      timestampQueue.clear();
      gyroYawQueue.clear();
    } finally {
      odometryLock.unlock();
    }
  }

  /**
   * Updates telemetry data in a thread-safe manner. It is important that this code is fast and does
   * not block as it runs at the frequency of the odometry thread.
   *
   * @param state Current state of the swerve drive
   */
  private void updateTelemetry(SwerveDriveState state) {
    odometryLock.lock();
    try {
      // Update module positions
      for (int i = 0; i < state.ModuleStates.length; i++) {
        drivePositionQueues.get(i).offer(state.ModulePositions[i].distanceMeters);
        steerPositionQueues.get(i).offer(state.ModulePositions[i].angle);
      }

      // Update gyro and timestamp data
      gyroYawQueue.offer(state.RawHeading);

      double currentTime = Timer.getFPGATimestamp();
      timestampQueue.offer(currentTime - (Utils.fpgaToCurrentTime(currentTime) - state.Timestamp));
    } finally {
      odometryLock.unlock();
    }
  }

  /** Starts the simulation thread with periodic updates. */
  private void startSimThread() {
    lastSimulationTime = Utils.getCurrentTimeSeconds();

    simulationNotifier =
        new Notifier(
            () -> {
              final double currentTime = Utils.getCurrentTimeSeconds();
              double deltaTime = currentTime - lastSimulationTime;
              lastSimulationTime = currentTime;

              // Update simulation with measured time delta and actual battery voltage
              updateSimState(deltaTime, RobotController.getBatteryVoltage());
            });

    simulationNotifier.startPeriodic(SIMULATION_LOOP_PERIOD);
  }

  @Override
  public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
    // Converts our WPILib timestamp to CTRE timestamp
    super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
  }

  @Override
  public Optional<Pose2d> samplePoseAt(double timestamp) {
    return super.samplePoseAt(Utils.fpgaToCurrentTime(timestamp));
  }

  @Override
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    // Converts our WPILib timestamp to CTRE timestamp
    super.addVisionMeasurement(
        visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
  }

  @Override
  public void updateModules(ModuleIOInputs[] inputs) {
    // Update modules with the given inputs
    for (int i = 0; i < Constants.PP_CONFIG.numModules; i++) {
      inputs[i] = updateModule(inputs[i], getModule(i));
    }
  }

  private ModuleIOInputs updateModule(ModuleIOInputs inputs, SwerveModule module) {
    // Get hardware objects
    TalonFX driveTalon = module.getDriveMotor();
    TalonFX turnTalon = module.getSteerMotor();
    CANcoder cancoder = module.getCANcoder();

    inputs.driveConnected = driveTalon.isConnected();
    inputs.drivePosition = driveTalon.getPosition().getValue();
    inputs.driveVelocity = driveTalon.getVelocity().getValue();
    inputs.driveAppliedVolts = driveTalon.getMotorVoltage().getValue();
    inputs.driveStatorCurrent = driveTalon.getStatorCurrent().getValue();
    inputs.driveSupplyCurrent = driveTalon.getSupplyCurrent().getValue();

    // Update turn inputs
    inputs.turnConnected = turnTalon.isConnected();
    inputs.turnEncoderConnected = cancoder.isConnected();
    inputs.turnAbsolutePosition =
        Rotation2d.fromRotations(cancoder.getAbsolutePosition().getValueAsDouble());
    inputs.turnPosition = Rotation2d.fromRotations(turnTalon.getPosition().getValueAsDouble());
    inputs.turnVelocity = turnTalon.getVelocity().getValue();
    inputs.turnAppliedVolts = turnTalon.getMotorVoltage().getValue();
    inputs.turnStatorCurrent = turnTalon.getStatorCurrent().getValue();
    inputs.turnSupplyCurrent = turnTalon.getSupplyCurrent().getValue();

    return inputs;
  }
}
