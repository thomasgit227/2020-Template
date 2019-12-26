/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team3039.robot.auto.routines;

import frc.team3039.robot.auto.AutoModeEndedException;
import frc.team3039.robot.auto.AutoRoutineBase;
import frc.team3039.robot.auto.actions.DriveTrajectory;
import frc.team3039.robot.auto.actions.SeriesAction;
import frc.team3039.robot.paths.TrajectoryGenerator;

/**
 * Add your docs here.
 */
public class AutoRocketElimination extends AutoRoutineBase {
    DriveTrajectory first_path;
    DriveTrajectory second_path;
    DriveTrajectory third_path;

    public AutoRocketElimination() {

        first_path = new DriveTrajectory(registerTrajectory(
                TrajectoryGenerator.getInstance().getTrajectorySet().level1StartReversedToRocketBack), true);

        second_path = new DriveTrajectory(
                registerTrajectory(TrajectoryGenerator.getInstance().getTrajectorySet().rocketBackToLoading), false);

        third_path = new DriveTrajectory(
                registerTrajectory(TrajectoryGenerator.getInstance().getTrajectorySet().loadingToRocketBack), false,
                false);
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new SeriesAction(first_path));

    }
}
