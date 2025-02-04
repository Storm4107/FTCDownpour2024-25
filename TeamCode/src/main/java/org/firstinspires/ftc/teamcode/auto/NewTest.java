package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SuperstructureSubsystem;


@Autonomous(name = "2025 - NewAuto", group = "Autonomous")
public class NewTest extends LinearOpMode {
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

            m_Superstructure.OpeningExtend();
            sleep(50);
            m_Superstructure.wallPickupPreset();
            sleep(50);
            m_Superstructure.pincher.setWristAngle(0);
            sleep(50);
            m_Superstructure.pincher.setPivotAngle(0);
            sleep(50);

            m_Drive.AutoDriveRC(36, 0, 7.65, 9.5, runtime);
            m_Drive.SetHeading(180, 9.65, 11.5, runtime);
            m_Drive.AutoDriveRC(0, 7, 11.65, 12.5, runtime);
            m_Drive.AutoDriveRC(45, 0, 12.65, 14.85, runtime);
            m_Drive.AutoDriveRC(8, 0, 15, 16.5, runtime);
            m_Superstructure.pincher.closeWithScheduler(16.5, 16.75, runtime);
            m_Superstructure.setAutoPosition(1810, 16.75, 19.25, runtime);
            m_Drive.AutoDriveRC(-10, -44, 16.85, 19, runtime);
            m_Drive.SetHeading(0, 19.15, 22, runtime);
            //Drop off
            m_Drive.AutoDriveRC(21, 0, 22.15, 23.25, runtime);
            m_Superstructure.setAutoPosition(1200, 23.25, 24.5, runtime);
            m_Superstructure.pincher.openWithScheduler(24.5, 24.75, runtime);
            m_Drive.SetHeading(0, 24.5, 25, runtime);
            m_Drive.AutoDriveRC(-26, -60, 25.15, 26.5, runtime);
            m_Superstructure.setAutoPosition(-100, 25, 26.5, runtime);
            m_Drive.SetHeading(180, 26.65, 28, runtime);
            m_Drive.AutoDriveRC(14, 0, 28.15, 29.15, runtime);
            m_Superstructure.pincher.closeWithScheduler(29.15, 30, runtime);


        }

    }
}
