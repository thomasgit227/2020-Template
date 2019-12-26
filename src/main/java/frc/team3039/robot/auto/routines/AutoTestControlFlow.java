package frc.team3039.robot.auto.routines;

import frc.team3039.robot.auto.AutoModeEndedException;
import frc.team3039.robot.auto.AutoRoutineBase;
import frc.team3039.robot.auto.actions.WaitAction;

public class AutoTestControlFlow extends AutoRoutineBase {

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("***** Starting test control flow mode");

        System.out.println("***** starting - first wait action");
        runAction(new WaitAction(10));
        System.out.println("***** done - first wait action ");

        waitForDriverConfirm(); // drivers do some manual stuff

        System.out.println("***** starting - second wait action");
        runAction(new WaitAction(10));
        System.out.println("***** done - second wait action ");

        waitForDriverConfirm(); // drivers do some manual stuff

        System.out.println("***** starting - third wait action");
        runAction(new WaitAction(10));
        System.out.println("***** done - third wait action ");
    }

}