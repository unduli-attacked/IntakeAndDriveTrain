package frc.robot.commands;

import org.ghrobotics.lib.mathematics.units.LengthKt;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class PWMHelperRocketAuto extends CommandGroup {
	public PWMHelperRocketAuto() {

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

		//FIXME generate trajectories
		// TimedTrajectory<Pose2dWithCurvature> traject = Trajectories.generatedLGTrajectories.get("habR" + " to " + "rocketRF"); //current trajectory from hashmap in Trajectories

		// addSequential(PWMDriveTrain.getInstance().followTrajectory(traject)); // drive to goal 

		addSequential(new RunIntake(-1, 1.5));

		addSequential(new PWMDriveDistanceTheThird(LengthKt.getFeet(3), true));
		// spline over to the rocket
		//FIXME generate trajectories
		// var rocketToLoading = Trajectories.generatedLGTrajectories.get("rocketRF to loadingR");
		// addSequential(PWMDriveTrain.getInstance().followTrajectory(rocketToLoading)); //drive to goal

		addSequential(new PWMDriveDistanceTheThird(LengthKt.getFeet(1.5), false));

		addSequential(new RunIntake(1, 1));

		//FIXME generate trajectories
		// var loadingToRocketFar = Trajectories.generatedLGTrajectories.get("loadingR to rocketRF");
		// addSequential(PWMDriveTrain.getInstance().followTrajectory(loadingToRocketFar)); //drive to goal
		addSequential(new RunIntake(-1, 1));

	}

	@Override
	protected boolean isFinished() {
		//FIXME thsi is correct, right?
		return false;
	}
}
