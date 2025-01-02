package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SuperstructureSubsystem;


@Autonomous(name = "2025 - Auto2Spec", group = "Autonomous")
public class Auto2Spec extends LinearOpMode {
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
        //init commands
        m_Superstructure.pincher.close();
        m_Superstructure.OpeningExtend();
        m_Superstructure.pincher.bucketHome();

        if (isStopRequested()) return;
        while (opModeIsActive()) {
            telemetry.addData("Current time", runtime.seconds());

            // scores preload Spec
            m_Superstructure.setAutoPosition(1785, 0, 4, runtime);
            m_Drive.AutoDriveRC(28, 3, 1, 3.5, runtime);
            m_Superstructure.setAutoPosition(1200, 3.5, 4.6, runtime);
            m_Superstructure.pincher.openWithScheduler(4.5, 4.6, runtime);

            // it will now pick a ground sample and bring it to the human player
            //m_Drive.AutoDriveRC(-8, 0, 7, 8, runtime);
            m_Superstructure.setAutoPosition(-100, 4.6, 10, runtime);
            m_Drive.AutoDriveRC(-12, -33, 4.75, 8, runtime);
            m_Drive.AutoDriveRC(36, 0, 8.15, 11, runtime);
            m_Drive.SetHeading(180, 11.15, 13, runtime);
            m_Drive.AutoDriveRC(0, 12, 13.15, 15, runtime);
            m_Drive.AutoDriveRC(48, 0, 15.15, 17.35, runtime);
            m_Drive.AutoDriveRC(11, 0, 17.45, 18.35, runtime);
            m_Superstructure.pincher.closeWithScheduler(18.45, 18.55, runtime);
            m_Superstructure.setAutoPosition(1785, 18.6, 25, runtime);
            m_Drive.AutoDriveRC(-10, -36.5, 19, 21, runtime);
            m_Drive.SetHeading(0, 21.15, 22.9, runtime);
            //Drop off
            m_Drive.AutoDriveRC(17, 0, 23, 24.65, runtime);
            m_Superstructure.setAutoPosition(1200, 24.75, 26, runtime);
            m_Superstructure.pincher.openWithScheduler(26, 26.1, runtime);
            m_Drive.AutoDriveRC(-40, -50, 26.5, 29, runtime);
            m_Superstructure.setAutoPosition(-100, 26.5, 29, runtime);

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
