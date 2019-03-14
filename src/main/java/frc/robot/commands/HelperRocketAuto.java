package frc.robot.commands;

import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature;
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Trajectories;

public class HelperRocketAuto extends CommandGroup {
	public HelperRocketAuto() {

		/*
		Thing do:
			Drive from starting location to the farside left rocket (I think, opposite the one we're doing, right?)
			Place hatch
			Drive to opposite loading station from us
			Pickup hatch
			Drive to nearside rocket
			Place hatch
		*/


		/* Get a trajectory to move to the cargo ship. THE ROBOT IS REVERSED */
		TimedTrajectory<Pose2dWithCurvature> traject = Trajectories.generatedLGTrajectories.get("habR" + " to " + "rocketRF"); //current trajectory from hashmap in Trajectories

		addSequential(TeleopDriving.drivetrain.followTrajectoryWithGear(traject, TrajectoryTrackerMode.RAMSETE, Gear.LOW, true)); //drive to goal

		// addParallel(new SuperstructureGoToState(fieldPositions.hatchMiddleGoal, iPosition.HATCH));
		addSequential(new FollowVisionTargetTheSecond(3.8));

		// addSequential(new DriveDistanceTheSecond(LengthKt.getFeet(0.4), false));

		// addSequential(new PrintCommand("GOT TO RUN INTAKE"));

		addSequential(new RunIntake(-1, 0, 1.5));

		// addSequential(new PrintCommand("GOT TO BACKING UP"));

		// back up 3 feet
		// addParallel(new JankyGoToState(iPosition.HATCH_GRAB_INSIDE_PREP)); // TODO fix this broken logic!
		addParallel(SequentialCommandFactory.getSequentialCommands(
			Arrays.asList(
				new ElevatorMove(iPosition.HATCH_GRAB_INSIDE.getElevator()),
				new JankyGoToState(iPosition.HATCH_GRAB_INSIDE)
			)
		));
		addSequential(new DriveDistanceTheSecond(LengthKt.getFeet(3), true));

		addSequential(new PrintCommand("GOT TO next spline"));

		// spline over to the rocket
		var rocketToLoading = Trajectories.generatedLGTrajectories.get("rocketRF to loadingR");
		addSequential(DriveTrain.getInstance().followTrajectoryWithGear(rocketToLoading, TrajectoryTrackerMode.RAMSETE, Gear.LOW, false)); //drive to goal

		// // addParallel(new SuperstructureGoToState(fieldPositions.hatchMiddleGoal, iPosition.HATCH));

		addSequential(new FollowVisionTargetTheSecond(4.5));

		addSequential(new DriveDistanceTheSecond(LengthKt.getFeet(1.5), false));

		addSequential(new RunIntake(1, 0, 1));

		// // addParallel(new LimeLight.SetLEDs(LimeLight.LEDMode.kOFF));
		addParallel(new JankyGoToState(fieldPositions.hatchLowGoal, iPosition.HATCH)); // move arm inside to prep state
		var loadingToRocketFar = Trajectories.generatedLGTrajectories.get("loadingR to rocketRF");
		addSequential(DriveTrain.getInstance().followTrajectoryWithGear(loadingToRocketFar, TrajectoryTrackerMode.RAMSETE, Gear.LOW, false)); //drive to goal
		addSequential(new FollowVisionTargetTheSecond(4.5));
		addSequential(new RunIntake(-1, 0, 1));


	}

	@Override
	protected boolean isFinished() {
		return false;
	}
}
