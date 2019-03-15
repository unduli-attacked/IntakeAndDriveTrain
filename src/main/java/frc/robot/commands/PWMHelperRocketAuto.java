package frc.robot.commands;

import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature;
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory;
import org.ghrobotics.lib.mathematics.units.LengthKt;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.PWMDriveTrain;
import frc.robot.Robot;
import frc.robot.Trajectories;

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

		char side = Robot.startingPos.getSelected().charAt(0);
		//FIXME assumes no middle hab start

		TimedTrajectory<Pose2dWithCurvature> traject = Trajectories.generatedLGTrajectories.get("hab"+side+" to rocket"+side+"F"); //current trajectory from hashmap in Trajectories

		addSequential(PWMDriveTrain.getInstance().followTrajectory(traject)); // drive to goal 

		addSequential(new RunIntake(-1, 1.5));

		addSequential(new PWMDriveDistanceTheThird(LengthKt.getFeet(3), true));
		// spline over to the rocket
		var rocketToLoading = Trajectories.generatedLGTrajectories.get("rocket"+side+"F to loading"+side);
		addSequential(PWMDriveTrain.getInstance().followTrajectory(rocketToLoading)); //drive to goal

		addSequential(new PWMDriveDistanceTheThird(LengthKt.getFeet(1.5), false));

		addSequential(new RunIntake(1, 1));

		//FIXME generate trajectories
		var loadingToRocketClose = Trajectories.generatedLGTrajectories.get("loading"+side+" to rocket"+side+"C");
		addSequential(PWMDriveTrain.getInstance().followTrajectory(loadingToRocketClose)); //drive to goal
		addSequential(new RunIntake(-1, 1));

	}

	@Override
	protected boolean isFinished() {
		//FIXME thsi is correct, right?
		return false;
	}
}
