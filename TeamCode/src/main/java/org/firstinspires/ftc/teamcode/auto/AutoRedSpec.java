package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SuperstructureSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurnSubsystem;


@Autonomous(name = "2025 - AutoRedSpec", group = "Autonomous")
public class AutoRedSpec extends LinearOpMode {
    //Instantiate mechanisms

    public SuperstructureSubsystem m_Superstructure;
    private MecanumDriveSubsystem m_Drive;

    public ElapsedTime runtime = new ElapsedTime();





    @Override
    public void runOpMode() {

        //Run when initializing
        m_Superstructure = new SuperstructureSubsystem(hardwareMap, telemetry);
        m_Drive = new MecanumDriveSubsystem(hardwareMap, telemetry);
        m_Drive.zeroPowerBrake();

        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.update();
            telemetry.addData("Auto", "Selected");
            m_Drive.zeroPowerBrake();
            runtime.reset();

            //init commands
            m_Superstructure.pincher.close();
            m_Superstructure.OpeningExtend();
            m_Superstructure.pincher.bucketHome();

        }
        waitForStart();
        m_Drive.resetDriveEncoders();

        if (isStopRequested()) return;
        while (opModeIsActive()) {
            telemetry.addData("Current time", runtime.seconds());

            // scores preload Spec
            m_Superstructure.setAutoPosition(1785, 0, 4, runtime);
            m_Drive.AutoDriveRC(28, 3, 2.5, 5, runtime);
            m_Superstructure.setAutoPosition(1200, 5, 7, runtime);
            m_Superstructure.pincher.openWithScheduler(6, 7, runtime);

            // it will now pick a ground sample and bring it to the human player
            m_Drive.AutoDriveRC(-8, 0, 7, 8, runtime);
            m_Superstructure.setAutoPosition(-100, 7, 10, runtime);
            m_Drive.AutoDriveRC(0, -33, 8.25, 11, runtime);
            m_Drive.AutoDriveRC(32, 0, 11.25, 14, runtime);
            m_Drive.SetHeading(180, 14.25, 15.5, runtime);
            m_Drive.AutoDriveRC(0, 12, 15.75, 17, runtime);
            m_Drive.AutoDriveRC(45, 0, 17.25, 18.75, runtime);
            m_Drive.AutoDriveRC(8, 0, 18.95, 20.5, runtime);
            m_Superstructure.pincher.closeWithScheduler(21, 22, runtime);
            m_Superstructure.setAutoPosition(1785, 22, 25, runtime);
            m_Drive.AutoDriveRC(-10, -36.5, 22, 25, runtime);
            m_Drive.SetHeading(0, 25.15, 26.5, runtime);
            //Drop off
            m_Drive.AutoDriveRC(17, 0, 26.65, 28.5, runtime);
            m_Superstructure.setAutoPosition(1200, 28.5, 29.5, runtime);
            m_Superstructure.pincher.openWithScheduler(29.7, 29.9, runtime);

        }
      //  sleep(1000);


        //m_Drive.SetHeading(188, 4);
        //m_Drive.AutoDriveRC(0,-30,3);

       // m_Drive.AutoDriveRC(30, 0, 3);

       // m_Drive.AutoDriveRC(0, -10, 3);

     //   sleep(2000);
       // m_Turn.Turn(.75, 950);
        //m_Drive.SetHeading(180,3);
      //  sleep(2000);




       // m_Drive.AutoDriveRC(47,0,6);

       // m_Superstructure.pincher.close();

       // sleep(500);

        //m_Superstructure.setAutoPosition(500, 1);
        //  m_Drive.AutoDriveRC(-10, 0, 2);

        //m_Drive.SetHeading(0, 3);
        //m_Drive.AutoDriveRC(0, 45, 2);

        //m_Superstructure.setAutoPosition(1785, 3);

       // m_Drive.AutoDriveRC(17, 0, 3);

       // m_Superstructure.setAutoPosition(1200, 1);

       // m_Superstructure.pincher.open();

       // m_Superstructure.setAutoPosition(0, 2);





        //m_Drive.AutoDriveRC(6,0,2);

        //sleep(500);



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
