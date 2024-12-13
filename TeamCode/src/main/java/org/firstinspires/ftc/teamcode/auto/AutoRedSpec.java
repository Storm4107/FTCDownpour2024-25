package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;



import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SuperstructureSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurnSubsystem;


@Autonomous(name = "2025 - AutoRedSpec", group = "Autonomous")
public class AutoRedSpec extends LinearOpMode {
    //Instantiate mechanisms


    private TurnSubsystem m_Turn;
    public SuperstructureSubsystem m_Superstructure;
    private MecanumDriveSubsystem m_Drive;




    @Override
    public void runOpMode() {



        //Run when initializing
        m_Superstructure = new SuperstructureSubsystem(hardwareMap, telemetry);
        m_Drive = new MecanumDriveSubsystem(hardwareMap, telemetry);
        m_Turn = new TurnSubsystem(hardwareMap, telemetry);




        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.update();
            telemetry.addData("Auto", "Selected");
        }
        waitForStart();

        if (isStopRequested()) return;

        //init commands
        m_Superstructure.pincher.close();
        m_Superstructure.OpeningExtend();
        m_Superstructure.pincher.bucketHome();

        // scores preload Spec

        m_Superstructure.setAutoPosition(1785, 2);
        m_Drive.AutoDriveRC(24, 0, 3);
        m_Superstructure.setAutoPosition(1200,1);
        m_Superstructure.pincher.open();

        // it will now pick a ground sample and bring it to the human player
        m_Drive.AutoDriveRC(-4,0,1);
        m_Superstructure.setAutoPosition(0, 3);
      //  sleep(1000);


        //m_Drive.SetHeading(188, 4);
        m_Drive.AutoDriveRC(0,-30,3);

        m_Drive.AutoDriveRC(24, 0, 3);

        m_Drive.AutoDriveRC(0, -15, 3);

     //   sleep(2000);
       // m_Turn.Turn(.75, 950);
        m_Drive.SetHeading(180,3);
      //  sleep(2000);




        m_Drive.AutoDriveRC(41,0,6);

        m_Superstructure.pincher.close();

        sleep(500);

        m_Superstructure.setAutoPosition(500, 1);
        m_Drive.AutoDriveRC(-10, 0, 2);

        m_Drive.SetHeading(0, 3);
        m_Drive.AutoDriveRC(0, 45, 2);

        m_Superstructure.setAutoPosition(1785, 3);

        m_Drive.AutoDriveRC(17, 0, 3);

        m_Superstructure.setAutoPosition(1200, 1);

        m_Superstructure.pincher.open();

        m_Superstructure.setAutoPosition(0, 2);





        //m_Drive.AutoDriveRC(6,0,2);

        sleep(500);



        //intake it here (i couldnt find the servo)
        //stop intake here
        //m_Drive.AutoDriveRC(0,-7,3);
        //m_Turn.Turn(.75, 950);
        //m_Drive.AutoDriveRC(8,0,2);
        //outtake here
        //m_Superstructure.pincher.bucketHome();

        //the robot will now pick up a new spec and score it

        //m_Drive.AutoDriveRC(24,0,4);


        //m_Drive.SetHeading(180,3);
        //m_Turn.Turn(.75, 950);
        //sleep(5000);

        //m_Drive.AutoDriveRC(27,0,4);
        //m_Superstructure.pincher.close();
        //m_Drive.AutoDriveRC(-4,0,3);
        //m_Drive.AutoDriveRC(0,-30, 7);
        //m_Turn.Turn(.75,950);
        //m_Superstructure.setAutoPosition(1785,5);
       // m_Drive.AutoDriveRC(26,0,5);
        //m_Superstructure.setAutoPosition(1200,3);
        //m_Superstructure.pincher.open();

    }
}
