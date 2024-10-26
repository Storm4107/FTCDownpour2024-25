package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;

@TeleOp(name = "DownpourTeleop24-25")
public class DownpourTeleop extends LinearOpMode {

    private MecanumDriveSubsystem m_Drive;
    private GamepadEx Driver;
    private GamepadEx Operator;



    @Override
    public void runOpMode() {
        m_Drive = new MecanumDriveSubsystem(hardwareMap, telemetry);

        Driver = new GamepadEx(gamepad1);

        Operator = new GamepadEx(gamepad2);

        //telemetry.update();
        waitForStart();

        while(opModeIsActive()) {
            //mecanum drivetrain code
            //LeftY is forward/backward, LeftX is side to side, RightX is rotate, DPAD_UP is speed management
            m_Drive.Drive(Driver.getLeftX(), Driver.getLeftY(), Driver.getRightX(), Driver.getButton(GamepadKeys.Button.DPAD_UP));

        }

    }

}
