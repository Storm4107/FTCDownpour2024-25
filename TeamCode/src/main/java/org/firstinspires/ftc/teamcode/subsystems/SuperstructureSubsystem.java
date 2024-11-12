package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

public class SuperstructureSubsystem {

    public SingleMotorArm Wrist;
    public DoubleMotorArm Arm;
    public LinearActuator Elevator;

    public PincherSubsystem pincher;

    private Motor leftArmMotor;
    private Motor rightArmMotor;

    private Motor wristMotor;

    private Motor elevatorMotor;

    private Servo leftServo;
    private Servo rightServo;
    private Telemetry telemetry;

    //private Telemetry telemetry;

    //Creates new superstructure (arm, elevator, wrist)
    public SuperstructureSubsystem(HardwareMap Map, Telemetry telemetry){

        //Create motor objects
        this.telemetry = telemetry;

        leftArmMotor = new Motor(Map, "leftArmMotor");
        rightArmMotor = new Motor(Map, "rightArmMotor");
        wristMotor = new Motor(Map, "wristMotor");
        elevatorMotor = new Motor(Map, "elevatorMotor");

        leftServo = Map.get(Servo.class, "leftServo");
        rightServo = Map.get(Servo.class, "rightServo");

        //Link motors to superstructure parts
        Wrist = new SingleMotorArm(
                wristMotor,
                Constants.SuperstructureConstants.wristGearRatio,
                false,
                Constants.SuperstructureConstants.wristPID);

        Arm = new DoubleMotorArm(
                leftArmMotor,
                rightArmMotor,
                Constants.SuperstructureConstants.armGearRatio,
                true,
                false,
                Constants.SuperstructureConstants.armPID);

        Elevator = new LinearActuator(
                elevatorMotor,
                Constants.SuperstructureConstants.elevatorCPI,
                true,
                Constants.SuperstructureConstants.elevatorPID);
    }

    public void enableDebug() {

        Wrist.setDebug();
        Arm.setDebug();
        Elevator.setDebug();
    }

    //Sample preset - Brings all mechanisms to 0
    /*public void zeroPreset() {

        Wrist.setAngle(0);
        Arm.setAngle(0);
        Elevator.setInches(0);
    }
*/
    //Sample preset - Brings all mechanisms to high drop-off
    public void highPreset() {

        Wrist.setAngle(1200);
        Arm.setAngle(2800);
        Elevator.setInches(1360);
    }

    //Sample preset - Brings all mechanisms to pickup
    public void pickupPreset() {

        Wrist.setAngle(1650);
        Arm.setAngle(0);
        Elevator.setInches(0);
    }

    //Sample preset - Brings all mechanisms to medium
    public void mediumPreset() {

        Wrist.setAngle(1130);
        Arm.setAngle(1970);
        Elevator.setInches(1360);
    }

    public void zeroPreset() {

        Elevator.setInches(0);
        pincher.retract();
    }

    //Sample preset - Brings all mechanisms to pickup
    public void groundPickupPreset() {

        Elevator.setInches(0);
        pincher.open();
        pincher.groundPickup();
    }

    public void wallPickupPreset() {

        Elevator.setInches(315);
        pincher.open();
        pincher.wallPickup();
    }

    //Sample preset - Brings all mechanisms to low bucket
    public void lowPreset() {

        Elevator.setInches(1600);
        pincher.scoreSample();
    }

    /**
     * Sets the arm into a manual input mode where the input can be toggled by button
     * @param input raw input - should be a joystick
     * @param armToggle button to input to arm
     * @param wristToggle button to input to wrist
     * @param elevatorToggle button to input to elevator
     */
    public void ManualInput(double input, boolean armToggle, boolean wristToggle, boolean elevatorToggle) {

        Arm.setOutput(armToggle ? input : 0);
        Wrist.setOutput(wristToggle ? input : 0);
        Elevator.setOutput(elevatorToggle ? input : 0);
    }

    public void periodic() {

        Arm.armPeriodic();
        Wrist.armPeriodic();
        Elevator.armPeriodic();
        telemetry.addData("Arm Angle", Arm.getAngle());
        telemetry.addData("Wrist Angle", Wrist.getAngle());
        telemetry.addData("Elevator Inches", Elevator.getInches());
    }
}
