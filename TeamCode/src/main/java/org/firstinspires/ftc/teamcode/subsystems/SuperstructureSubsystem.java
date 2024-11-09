package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.hardware.DoubleMotorArm;
import org.firstinspires.ftc.teamcode.hardware.DoubleMotorLinearActuator;
import org.firstinspires.ftc.teamcode.hardware.DoubleServoPincher;
import org.firstinspires.ftc.teamcode.hardware.LinearActuator;
import org.firstinspires.ftc.teamcode.hardware.ServoActuator;
import org.firstinspires.ftc.teamcode.hardware.SingleMotorArm;

public class SuperstructureSubsystem {

    public DoubleMotorLinearActuator Elevator;

    public PincherSubsystem pincher;

    private Motor elevatorMotor1;
    private Motor elevatorMotor2;

    private Servo lateratorServo;
    public ServoActuator extendo;

    private Telemetry telemetry;

    ElapsedTime runtime;

    //Creates new superstructure (arm, elevator, wrist)
    public SuperstructureSubsystem(HardwareMap Map, Telemetry telemetry){

        //Create motor objects
        this.telemetry = telemetry;

        elevatorMotor1 = new Motor(Map, "elevatorMotor1");
        elevatorMotor2 = new Motor(Map, "elevatorMotor2");

        lateratorServo = Map.get(Servo.class, "lateratorServo");

        extendo = new ServoActuator(lateratorServo);

        pincher = new PincherSubsystem(Map);

        //Link motors to superstructure parts
        Elevator = new DoubleMotorLinearActuator(
                elevatorMotor1,
                elevatorMotor2,
                Constants.SuperstructureConstants.elevatorCPI,
                true,
                false,
                Constants.SuperstructureConstants.elevatorPID);
    }

    public void enableDebug() {

        Elevator.setDebug();
    }

    //Sample preset - Brings all mechanisms to 0
    public void zeroPreset() {

        Elevator.setInches(0);
        extendo.setServos(1);
        pincher.retract();
    }

    //Sample preset - Brings all mechanisms to pickup
    public void groundPickupPreset() {

        Elevator.setInches(0);
        extendo.setServos(-1);
        pincher.open();
        pincher.groundPickup();
    }

    public void wallPickupPreset() {

        Elevator.setInches(315);
        extendo.setServos(.7);
        pincher.open();
        pincher.wallPickup();
    }

    //Sample preset - Brings all mechanisms to low bucket
    public void lowPreset() {

        Elevator.setInches(1600);
        extendo.setServos(.7);
        pincher.scoreSample();
    }

    //Sample preset - Brings all mechanisms to high bucket
    public void highPreset() {

        Elevator.setInches(2400);
        extendo.setServos(.7);
        pincher.scoreSample();
    }

    /**
     * Sets the Elevator/laterator into a manual input mode where the input can be toggled by button
     * @param input1 raw input to the elevator - should be a joystick
     */
    public void ManualInput(double input1) {

        Elevator.setOutput(input1 * 1);
        telemetry.addData("Elevator tick", Elevator.motor1.getCurrentPosition());
    }

    public void periodic() {

        Elevator.Periodic();
        telemetry.addData("Elevator Inches", Elevator.getInches());
    }

    public void setAutoPosition(double ElevatorInches, double TimeoutS) {
            runtime.reset();

            Elevator.setInches(ElevatorInches);

            while((runtime.seconds() < TimeoutS) &&
                    !Elevator.atSetpoint()) {
                //Periodic
                //actually drives the Superstructure.
                Elevator.Periodic();
                telemetry.addData("SUPERSTRUCTURE STATUS", "RUNNING");
                telemetry.addData("Elevator inches:", Elevator.getInches());
                telemetry.update();
            }
    }
}
