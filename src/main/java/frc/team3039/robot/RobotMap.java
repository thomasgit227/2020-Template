package frc.team3039.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
	// USB Port IDs
	public static final int DRIVER_JOYSTICK_1_USB_ID = 0;
	public static final int OPERATOR_JOYSTICK_1_USB_ID = 1;

	// Motors
	public static final int DRIVETRAIN_RIGHT_MOTOR1_CAN_ID = 0;
	public static final int DRIVETRAIN_RIGHT_MOTOR2_CAN_ID = 1;
	public static final int DRIVETRAIN_RIGHT_MOTOR3_CAN_ID = 2;

	public static final int DRIVETRAIN_LEFT_MOTOR1_CAN_ID = 15;
	public static final int DRIVETRAIN_LEFT_MOTOR2_CAN_ID = 14;
	public static final int DRIVETRAIN_LEFT_MOTOR3_CAN_ID = 13;


}
