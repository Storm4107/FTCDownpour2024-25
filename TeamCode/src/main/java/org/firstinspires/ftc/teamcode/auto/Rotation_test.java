package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SuperstructureSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurnSubsystem;


@Autonomous(name = "2025 - AutoRedSpec", group = "Autonomous")
public class Rotation_test extends LinearOpMode {
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
        m_Drive.zeroPowerBrake();



        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.update();
            telemetry.addData("Auto", "Selected");
            telemetry.addData("Counts",m_Drive.getHeading());
            Telemetry.addData("test", String.valueOf(Math.IEEEremainder(m_Drive.imu.getAbsoluteHeading(), 360)));
            m_Drive.zeroPowerBrake();

        }
        waitForStart();

        if (isStopRequested()) return;

        //init commands
        m_Superstructure.pincher.close();
        m_Superstructure.OpeningExtend();
        m_Superstructure.pincher.bucketHome();

        // scores preload Spec


       // m_Drive.AutoDriveRC(0, -10, 3);

     //   sleep(2000);
       // m_Turn.Turn(.75, 950);
        m_Drive.SetHeading(190,3);

    }
}
