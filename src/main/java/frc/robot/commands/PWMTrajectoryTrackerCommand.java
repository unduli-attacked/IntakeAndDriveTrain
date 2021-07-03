package frc.robot.commands;

import java.util.function.Supplier;

import org.ghrobotics.lib.debug.LiveDashboard;
import org.ghrobotics.lib.mathematics.twodim.control.TrajectoryTracker;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature;
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedEntry;
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory;
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TrajectorySamplePoint;
import org.ghrobotics.lib.mathematics.units.Length;
import org.ghrobotics.lib.mathematics.units.TimeUnitsKt;
import org.ghrobotics.lib.subsystems.drive.TrajectoryTrackerOutput;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.DriveTrain;
import frc.robot.PWMDriveTrain;
import frc.robot.Trajectories;

// @SuppressWarnings({"WeakerAccess", "unused"})
public class PWMTrajectoryTrackerCommand extends Command {
	private TrajectoryTracker trajectoryTracker;
	private Supplier<TimedTrajectory<Pose2dWithCurvature>> trajectorySource;
	private PWMDriveTrain driveBase;
	private boolean reset;
	private TrajectoryTrackerOutput output;
	// TODO make sure that this fabled namespace collision doesn't happen on
	// Shuffleboard
	Length mDesiredLeft;
	Length mDesiredRight;
	double mCurrentLeft;
	double mCurrentRight;

	Notifier mUpdateNotifier;

	public PWMTrajectoryTrackerCommand(PWMDriveTrain driveBase,
			Supplier<TimedTrajectory<Pose2dWithCurvature>> trajectorySource) {
		this(driveBase, trajectorySource, false);
	}

	public PWMTrajectoryTrackerCommand(PWMDriveTrain driveBase,
			Supplier<TimedTrajectory<Pose2dWithCurvature>> trajectorySource, boolean reset) {
		this(driveBase, DriveTrain.getInstance().getTrajectoryTracker(), trajectorySource, reset);
	}

	public PWMTrajectoryTrackerCommand(PWMDriveTrain driveBase, TrajectoryTracker trajectoryTracker,
			Supplier<TimedTrajectory<Pose2dWithCurvature>> trajectorySource, boolean reset) {
		requires(driveBase);
		this.driveBase = driveBase;
		this.trajectoryTracker = trajectoryTracker;
		this.trajectorySource = trajectorySource;
		this.reset = reset;
		// this.mModel = driveBase.getDifferentialDrive();
	}

	@Override
	protected void initialize() {
		LiveDashboard.INSTANCE.setFollowingPath(false);

		if (trajectorySource == null) {
			System.out.println("Sadly the trajectories are not generated. the person responsible for the trajectories has been sacked.");
			Trajectories.generateAllTrajectories();
		}

		System.out.println("get: " + trajectorySource.get().getFirstState().getState().getCurvature().toString());

		trajectoryTracker.reset(this.trajectorySource.get());

		System.out.println("first pose: " + trajectorySource.get().getFirstState().getState().getPose().getTranslation().getX().getFeet());

		if (reset == true) {
			DriveTrain.getInstance().getLocalization().reset(trajectorySource.get().getFirstState().getState().getPose());
		}

		System.out.println("desired linear, real linear");

		LiveDashboard.INSTANCE.setFollowingPath(true);

		mUpdateNotifier = new Notifier(() -> {
			output = trajectoryTracker.nextState(driveBase.getRobotPosition(), TimeUnitsKt.getSecond(Timer.getFPGATimestamp()));

			TrajectorySamplePoint<TimedEntry<Pose2dWithCurvature>> referencePoint = trajectoryTracker.getReferencePoint();
			if (referencePoint != null) {
				Pose2d referencePose = referencePoint.getState().getState().getPose();

				LiveDashboard.INSTANCE.setPathX(referencePose.getTranslation().getX().getFeet());
				LiveDashboard.INSTANCE.setPathY(referencePose.getTranslation().getY().getFeet());
				LiveDashboard.INSTANCE.setPathHeading(referencePose.getRotation().getRadian());
			}
			// Logger.log("Linear: " + output.getLinearVelocity().getValue() + " Angular: " + output.getAngularVelocity().getValue() );
			driveBase.setOutput(output);
		});
		mUpdateNotifier.startPeriodic(0.01);
	}

	@Override
	protected void end() {
		mUpdateNotifier.stop();
		driveBase.stop();
		LiveDashboard.INSTANCE.setFollowingPath(false);
	}

	@Override
	protected boolean isFinished() {
		return trajectoryTracker.isFinished();
	}

	public TimedTrajectory<Pose2dWithCurvature> getTrajectory() {
		return this.trajectorySource.get();
	}

}
