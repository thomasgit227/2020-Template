# 2020-Template 
## Project Description

The goal of this project is to simplify, organize, and consolidate some very complex autonomous code to work for your team with minimal changes. Inspired from [@Team254](https://github.com/Team254) and some of the work that I did on [@Team3310](https://github.com/Team3310), this template will allow you to start working with effective path following autonomous code for the 2020 FRC Season. I worked to make this as comprehensible as possible so if you have any feedback I am more than welcome to it! 

The code is divided into several packages, each responsible for a different aspect of robot function. Some of the packages you will see in the project overlap between this [project](https://github.com/JoshLew7/2020-Template) and [Team254](https://github.com/Team254/FRC-2018-Public#frc-2018) this repository so this README will highlight some of the changes that were made in the project. I highly recommend you go read 254's README for more information about some of these packages along with additional information found in that class's Java files.

## Project Features
* Path following with a nonlinear feedback controller and splines
    
    To control autonomous driving, the robot utilizes a [nonlinear feedback controller](https://github.com/JoshLew7/2020-Templete/blob/master/src/main/java/frc/team3039/robot/planners/DriveMotionPlanner.java) and drives paths constructed of [Quintic Hermite Splines](https://github.com/JoshLew7/2020-Templete/blob/master/src/main/java/frc/team3039/utility/lib/spline/QuinticHermiteSpline.java).
    
* Path generation and visualization via Java app

    [Hawk Path](https://github.com/JoshLew7/2020-Template/tree/master/src/main/java/frc/team3039/path), an external Java webapp, is used for fast and simple path creation. The app allows a user to create and visualize autonomous paths.

## Getting Started
  ### General
  * **Clone** this repository or use the **Use this Template** button to get a version of this repository on your computer.  
  *  **Make sure that you have a folder on your computer that is linked to your repository if you are using source control.**
  * Run *./gradlew* to download Gradle and needed FRC libraries
  * Run *./gradlew tasks* to see available build options
  
  ### VSCode 
    Visual Studio Code (Official IDE)
    Get the WPILib extension for easiest use from the VSCode Marketplace - Requires Java 11 or greater
    In .vscode/settings.json, set the User Setting, java.home, to the correct directory pointing to your JDK 11 dir
  
  ### Eclipse
    Run ./gradlew eclipse
    Open Eclipse and go to "File > Open Projects" from "File System..."
    Set the import source to the 2020-Template folder then click finish
  ### IntelliJ
    If you used the Use This Template button, you can open the repository through the Open Project through version control option
    or
    Run ./gradlew idea
    Open the 2020-Template folder
    Building/Deploying to the Robot
    Run ./gradlew build to build the code. Use the --info flag for more details
    Run ./gradlew deploy to deploy to the robot in Terminal (Mac) or Powershell (Windows)
    
## Package Description 

 ### **Class Addition**
  
 ### ~~Class Removal~~
  
* frc.team3039.robot
 
  Holds all the folders that control robot actions, along with files such as the numerical constants used throughout the code, and classes like [Robot](https://github.com/JoshLew7/2020-Template/blob/master/src/main/java/frc/team3039/robot/Robot.java) which facilitates robot control. 
  
  **AutoRoutineSelector**   ~~AutoModeSelector~~
  
  * [AutoRoutineSelector](https://github.com/JoshLew7/2020-Template/blob/master/src/main/java/frc/team3039/robot/AutoRoutineSelector.java) handles all of the autonomous selection and setup. You select a StartingPosition, a DesiredMode which allows for different actions depending on the mode you are running your auto in instead of to represent a routine i.e. *you may zero you elevator in teleOp Init if you are running in test mode but you may want to hold it's current position if you are running in competition mode*, and the routine you would like to run. Every time a change is made on the smart dashboard this class's methods will update to set your auto to your desired selection. 
  
    *Removed mFeildState (2018 Game Specifics), Renamed for simplicity*

  
* frc.team3039.robot.auto

  Handles the execution of autonomous routines and contains *actions* and **routines** packages
  
  **AutoRoutineBase**   ~~AutoModeBase~~
  
  * [AutoRoutineBase](https://github.com/JoshLew7/2020-Template/blob/master/src/main/java/frc/team3039/robot/auto/AutoRoutineBase.java) is the basis of the robot's autonomous routines. A LazyLoadTracketory class was implemented in the class. The LazyLoadTrajectory class allows you to only generate trajectories when you select your autonomous routine instead of generating all of the trajectories when some may be unneeded for a specific match. This was added to limit processing over the FMS and remove unneeded action.
  
    *Renamed because it sets up the autonomous routines (no longer known as a mode)*

* frc.team3039.robot.auto.actions

  * Includes all actions used during the autonomous period, which all share a common interface, Action (also in this package). Examples include auto vision and driving a trajectory. Routines interact with the subsystems, which interact with the hardware.
  
  **CharacterizeDrivebase**
  
  * [CharacterizeDrivebase](https://github.com/JoshLew7/2020-Template/blob/master/src/main/java/frc/team3039/robot/auto/actions/CharacterizeDrivebase.java) combines the collect acceleration and velocity data actions along with CalculateCharacteriztion to return the *kv* and *ka* values that will be used to tune your robot constants. 
  
  **CalculateCharacterization**
  
  * [CalculateCharacterization](https://github.com/JoshLew7/2020-Template/blob/master/src/main/java/frc/team3039/robot/auto/actions/CalculateCharacterization.java) prints the velocity and acceleration values calculated from the CharacterizeDrivebase Action.
  
  **WaitUntilCrossXBoundary** ~~WaitUntilCrossXBoundaryCommand~~
  
  
  [WaitUntilCrossXBoundary](https://github.com/JoshLew7/2020-Template/blob/master/src/main/java/frc/team3039/robot/auto/actions/WaitUntilCrossXBoundary.java)  has added functionality for when the robots x is either moving positively or negatively into a certain boundary. *i.e. Positive (moving towards the middle of the field) Negative (moving toward the alliance wall)*
   
* ~~com.team254.frc2018.auto.creators~~

   ~~Contains all the auto mode creators, which select the correct auto mode to run based on user input and FMS data.~~

  *Removed because this was 2018 game-specific functionality*

* frc.team3039.robot.auto.routines ~~com.team254.frc2018.auto.modes~~

  It contains all autonomous routines. Autonomous routines are set up similar to command groups where actions run in a specified order 
  
  *Renamed for simplicity*
  
 **This will not give you perfect understanding of the code, you will have to look at it to understand but this should be a guide to help you along.**
  
## Author

* **Joshua Lewis** - *Initial work* - [Repositories](https://github.com/JoshLew7)

## Acknowledgments

* [Team254: The Chessy Poof](https://github.com/Team254)
* [Team3310: Brian Selle](https://github.com/BrianSelle)

