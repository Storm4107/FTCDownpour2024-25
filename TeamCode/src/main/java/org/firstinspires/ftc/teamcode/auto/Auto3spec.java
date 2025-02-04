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
            m_Superstructure.setAutoPosition(1785, 0, 3, runtime);
            m_Drive.AutoDriveRC(28, 3, 1, 3, runtime);
            m_Superstructure.setAutoPosition(1200, 3.15, 4.35, runtime);
            m_Superstructure.pincher.openWithScheduler(4, 4.25, runtime);

            // it will now pick a ground sample and bring it to the human player
            m_Superstructure.setAutoPosition(-100, 4.35, 7.5, runtime);
            m_Drive.AutoDriveRC(-12, -31.5, 4.75, 6.5, runtime);
            m_Drive.AutoDriveRC(36, 0, 6.6, 7.95, runtime);
            m_Drive.SetHeading(180, 8, 9.35, runtime);

            m_Drive.AutoDriveRC(0, 12, 9.45, 10.3, runtime);
            m_Drive.AutoDriveRC(48, 0, 10.4, 12.35, runtime);
            m_Drive.AutoDriveRC(11, 0, 12.4, 12.95, runtime);
            m_Superstructure.pincher.closeWithScheduler(13, 13.55, runtime);
            m_Superstructure.setAutoPosition(1785, 13.6, 16.3, runtime);
            m_Drive.AutoDriveRC(-10, -36.5, 16.4, 17.85, runtime);
            m_Drive.SetHeading(0, 17.9, 19.1, runtime);
            //Drop off
            m_Drive.AutoDriveRC(15, 0, 19.2, 19.45, runtime);
            m_Superstructure.setAutoPosition(1200, 19.5, 21.7, runtime);
            m_Superstructure.pincher.openWithScheduler(20.6, 20.85, runtime);
            m_Drive.SetHeading(0, 21.8, 22.3, runtime);
            m_Drive.AutoDriveRC(-28, -60, 22.4, 23.4, runtime);
            m_Superstructure.setAutoPosition(-100, 23.45, 24.8, runtime);
            m_Drive.SetHeading(180, 24.9, 26, runtime);
            m_Drive.AutoDriveRC(14, 0, 26.1, 27, runtime);
            m_Superstructure.pincher.closeWithScheduler(27.1, 27.35, runtime);
            //Score 3rd Spec
            m_Drive.SetHeading(0,27.45,28,runtime);
            m_Drive.AutoDriveRC(28,62,28.1,29,runtime);

        }

    }
}
