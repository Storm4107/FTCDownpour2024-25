package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SuperstructureSubsystem;


@Autonomous(name = "2025 - Auto3spec", group = "Autonomous")
public class Auto3spec extends LinearOpMode {
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
            m_Superstructure.setAutoPosition(1785, 0, 1.9, runtime);
            m_Drive.AutoDriveRC(28, 3, 1, 1.9, runtime);
            m_Superstructure.setAutoPosition(1200, 2, 3, runtime);
            m_Superstructure.pincher.openWithScheduler(2.3, 2.55, runtime);

            // it will now pick a ground sample and bring it to the human player
            m_Superstructure.setAutoPosition(-100, 3.1, 4, runtime);
            m_Drive.AutoDriveRC(-12, -31, 3.4, 4.7, runtime);
            m_Drive.AutoDriveRC(30, 0, 4.8, 5.8, runtime);
            m_Drive.SetHeading(180, 5.9, 7.4, runtime);

            m_Drive.AutoDriveRC(0, 7, 7.5, 8, runtime);
            m_Drive.AutoDriveRC(42, 0, 8.1, 9.2, runtime);
            m_Drive.AutoDriveRC(5, 0, 9.3, 9.7, runtime);
            m_Superstructure.pincher.closeWithScheduler(9.8, 10.05, runtime);
            m_Superstructure.setAutoPosition(1800, 10.1, 12.1, runtime);
            m_Drive.AutoDriveRC(-10, -40, 12.2, 13.5, runtime);
            m_Drive.SetHeading(0, 13.6, 15, runtime);
            //Drop off

            m_Drive.AutoDriveRC(15, 0, 15.1, 16, runtime);
            m_Superstructure.setAutoPosition(1200, 16.1, 17.1, runtime);
            m_Superstructure.pincher.openWithScheduler(16.3, 16.55, runtime);
            m_Drive.SetHeading(0, 17.2, 17.7, runtime);
            m_Superstructure.setAutoPosition(-100, 17.8, 18.8, runtime);
            m_Drive.AutoDriveRC(-15, -50, 18.9, 20.2, runtime);

            m_Drive.SetHeading(180, 20.3, 21.5, runtime);
            m_Drive.AutoDriveRC(14, 0, 21.6, 22.6, runtime);
            m_Superstructure.pincher.closeWithScheduler(22.7, 22.95, runtime);
            //Score 3rd Spec
            m_Superstructure.setAutoPosition(1800,23,25,runtime);
            m_Drive.AutoDriveRC(-15,-35,25.1,26.4,runtime);
            m_Drive.SetHeading(0,26.5,27.5,runtime);
            m_Drive.AutoDriveRC(7,0,27.6,28.3,runtime);
            m_Superstructure.setAutoPosition(1200,28.4,29.4,runtime);
            m_Superstructure.pincher.openWithScheduler(28.6,28.85,runtime);
            m_Superstructure.setAutoPosition(-100,29,30,runtime);




        }

    }
}
