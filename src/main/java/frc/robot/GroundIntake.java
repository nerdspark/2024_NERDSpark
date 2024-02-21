package frc.robot;

// public class GroundIntake extends SequentialCommandGroup {
//         public GroundIntake(
//             FourBar FourBar,
//             Intake intake,
//             Shooter shooter
//             NoteSensor noteSensor) {
//         addCommands(
//                 new InstantCommand(() -> armSubsystem.setArmPositionState(ArmSubsystem.ArmPosition.GROUND_PICKUP)),
//                 new MoveGripperCommand(gripperSubsystem, armSubsystem, MoveGripperCommand.GripperState.CLOSED),
//                 new ParallelCommandGroup(
//                         new MoveArmCommand(
//                                 armSubsystem,
//                                 ArmConstants.groundPickupPosition.armCmdPos(),
//                                 ArmConstants.groundPickupPosition.smartMotionMaxVel(),
//                                 ArmConstants.groundPickupPosition.smartMotionMaxAccel()),
//                         new MoveElevatorCommand(elevatorSubsystem,
// ArmConstants.groundPickupPosition.elevatorCmdPos()),
//                         new WaitCommand(0.3)
//                                 .andThen(new MoveWristCommand(
//                                         wristSubsystem, ArmConstants.groundPickupPosition.wristCmdPos()))),
//                 new MoveGripperCommand(gripperSubsystem, armSubsystem, MoveGripperCommand.GripperState.OPENED));
//     }

// }
