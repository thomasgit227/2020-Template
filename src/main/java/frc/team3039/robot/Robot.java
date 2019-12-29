/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team3039.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team3039.robot.AutoRoutineSelector.DesiredMode;
import frc.team3039.robot.auto.AutoModeExecutor;
import frc.team3039.robot.auto.AutoRoutineBase;
import frc.team3039.robot.loops.Looper;
import frc.team3039.robot.paths.TrajectoryGenerator;
import frc.team3039.robot.subsystems.Drive;
import frc.team3039.robot.subsystems.Drive.DriveControlMode;
import frc.team3039.robot.subsystems.RobotStateEstimator;
import frc.team3039.utility.CrashTracker;
import frc.team3039.utility.lib.geometry.Pose2d;

import java.util.Arrays;
import java.util.Optional;

public class Robot extends TimedRobot {
	// public static OI oi;

	// Control looper
	private Looper mEnabledLooper = new Looper();
	private Looper mDisabledLooper = new Looper();

	private TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();
	private AutoRoutineSelector mAutoRoutineSelector = new AutoRoutineSelector();
	private AutoModeExecutor mAutoModeExecutor;
	private DesiredMode mOperationMode;

	private final SubsystemManager mSubsystemManager = new SubsystemManager(
			Arrays.asList(RobotStateEstimator.getInstance(), Drive.getInstance()));

	// Declare subsystems
	private Drive mDrive = Drive.getInstance();
	private RobotStateEstimator mRobotStateEstimator = RobotStateEstimator.getInstance();

	public Robot() {
		CrashTracker.logRobotConstruction();
	}

	public void zeroAllSensors() {
		mDrive.zeroSensors();
	}

	// Called at the start of connection
	@Override
	public void robotInit() {
		// oi = OI.getInstance();
		try {
			// UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
			// camera.setVideoMode(VideoMode.PixelFormat.kMJPEG, 320, 240, 15);
			// MjpegServer cameraServer = new MjpegServer("serve_USB Camera 0",
			// Constants.kCameraStreamPort);
			// cameraServer.setSource(camera);

			CrashTracker.logRobotInit();

			mSubsystemManager.registerEnabledLoops(mEnabledLooper);
			mSubsystemManager.registerDisabledLoops(mDisabledLooper);

			mTrajectoryGenerator.generateTrajectories();
			mAutoRoutineSelector.updateModeCreator();

			zeroAllSensors();

		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	// Called every loop for all modes
	public void robotPeriodic() {
		// outputToSmartDashboard();
	}

	// Called once when is disabled
	@Override
	public void disabledInit() {
		SmartDashboard.putString("Match Cycle", "DISABLED");

		try {
			CrashTracker.logDisabledInit();
			mEnabledLooper.stop();
			if (mAutoModeExecutor != null) {
				mAutoModeExecutor.stop();
			}

			// mInfrastructure.setIsDuringAuto(true);
			mDrive.zeroSensors();
			RobotState.getInstance().reset(Timer.getFPGATimestamp(), Pose2d.identity());

			// Reset all auto mode state.
			mAutoRoutineSelector.reset();
			mAutoRoutineSelector.updateModeCreator();
			mAutoModeExecutor = new AutoModeExecutor();

			mDisabledLooper.start();

		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	// Called constantly when the robot is disabled
	@Override
	public void disabledPeriodic() {
		SmartDashboard.putString("Match Cycle", "DISABLED");

		try {
			mOperationMode = mAutoRoutineSelector.getDesiredMode();
			outputToSmartDashboard();
			mAutoRoutineSelector.updateModeCreator();

			Optional<AutoRoutineBase> autoMode = mAutoRoutineSelector.getAutoMode();
			if (autoMode.isPresent() && autoMode.get() != mAutoModeExecutor.getAutoMode()) {
				System.out.println("Set auto mode to: " + autoMode.get().getClass().toString());
				mAutoModeExecutor.setAutoMode(autoMode.get());
			}
			System.gc();
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	// Called once at the start of auto
	@Override
	public void autonomousInit() {
		SmartDashboard.putString("Match Cycle", "AUTONOMOUS");

		try {
			CrashTracker.logAutoInit();
			mDisabledLooper.stop();

			RobotState.getInstance().reset(Timer.getFPGATimestamp(), Pose2d.identity());

			mDrive.zeroSensors();

			mAutoModeExecutor.start();

			mEnabledLooper.start();

		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	// Called constantly through auton
	@Override
	public void autonomousPeriodic() {
		SmartDashboard.putString("Match Cycle", "AUTONOMOUS");

		outputToSmartDashboard();
		try {
			System.out.println("Error in Autonomous Periodic");

		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}

	}

	// Called once at the start of teleOp
	@Override
	public void teleopInit() {
		SmartDashboard.putString("Match Cycle", "TELEOP");

		try {
			CrashTracker.logTeleopInit();
			mDisabledLooper.stop();
			if (mAutoModeExecutor != null) {
				mAutoModeExecutor.stop();
			}

			mDrive.setControlMode(DriveControlMode.JOYSTICK);

			mDrive.endGyroCalibration();

			if (mOperationMode == DesiredMode.COMPETITION) {
				System.out.println("TeleOp Init Competition Mode");
			}

			if (mOperationMode != DesiredMode.COMPETITION) {
				System.out.println("TeleOp Init Test Modes");
			}

			RobotState.getInstance().reset(Timer.getFPGATimestamp(), Pose2d.identity());
			mEnabledLooper.start();

		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	// Called constantly through TeleOp
	@Override
	public void teleopPeriodic() {
		SmartDashboard.putString("Match Cycle", "TELEOP");

		try {

			outputToSmartDashboard();
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void testInit() {
		try {
			CrashTracker.logTestInit();
			System.out.println("Starting check systems.");

			mDisabledLooper.stop();
			mEnabledLooper.stop();

			if (mSubsystemManager.checkSubsystems()) {
				System.out.println("ALL SYSTEMS PASSED");
			} else {
				System.out.println("CHECK ABOVE OUTPUT SOME SYSTEMS FAILED!!!");
			}
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	public void outputToSmartDashboard() {
		mDrive.outputTelemetry(mOperationMode);
		mRobotStateEstimator.outputTelemetry(mOperationMode);
	}
}
