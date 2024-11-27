package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;



import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SuperstructureSubsystem;



@Autonomous(name = "2025 - AutoRedSpec", group = "Autonomous")
public class AutoRedSpec extends LinearOpMode {
    //Instantiate mechanisms




    public SuperstructureSubsystem m_Superstructure;
    private MecanumDriveSubsystem m_Drive;




    @Override
    public void runOpMode() {



        //Run when initializing
        m_Superstructure = new SuperstructureSubsystem(hardwareMap, telemetry);
        m_Drive = new MecanumDriveSubsystem(hardwareMap, telemetry);




        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.update();
            telemetry.addData("Auto", "Selected");
        }
        waitForStart();

        if (isStopRequested()) return;

        //init commands

        m_Superstructure.OpeningExtend();
        m_Superstructure.pincher.close();
        m_Superstructure.pincher.bucketHome();

        // scores preload Spec

        m_Superstructure.setAutoPosition(1785, 5);
        m_Drive.AutoDriveRC(30, 0, 5);
        m_Superstructure.setAutoPosition(1200,3);
        m_Superstructure.pincher.open();

        // it will now pick a ground sample and bring it to the human player

        m_Superstructure.setAutoPosition(0, 3);
        m_Drive.AutoDriveRC(-4,0,3);
        m_Drive.SetHeading(-90, 4);
        m_Superstructure.pincher.groundPickup();
        m_Drive.AutoDriveRC(-29,0,5);
        m_Drive.AutoDriveRC(0,-13,3);
        m_Drive.AutoDriveRC(-3,0,2);
        //intake it here (i couldnt find the servo)
        //stop intake here
        m_Drive.AutoDriveRC(3,0,2);
        m_Drive.SetHeading(90, 3);
        m_Drive.AutoDriveRC(-34,0,6);
        //outtake here
        m_Superstructure.pincher.bucketHome();

        //the robot will now pick up a new spec and score it

        m_Drive.AutoDriveRC(24,0,4);
        m_Drive.SetHeading(180,3);
        sleep(5000);
        m_Drive.AutoDriveRC(27,0,4);
        m_Superstructure.pincher.close();
        m_Drive.AutoDriveRC(-4,0,3);
        m_Drive.AutoDriveRC(0,-30, 7);
        m_Drive.SetHeading(180,3);
        m_Superstructure.setAutoPosition(1785,5);
        m_Drive.AutoDriveRC(26,0,5);
        m_Superstructure.setAutoPosition(1200,3);
        m_Superstructure.pincher.open();


    }
}
