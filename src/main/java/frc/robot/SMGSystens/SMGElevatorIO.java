package frc.robot.SMGSystens;

public interface SMGElevatorIO {

   class SMGElevatorInputs {

        public double ElevatorPos = 0.0;
        public double RLPostion = 0.0;
        public double FLPosition = 0.0;
        public double RLVoltage = 0.0;
        public double FLVoltage = 0.0;

    }

    public default void setVolatgeElevator(double voltage){}

    public default void updateInputs(SMGElevatorInputs elevatorInputs){}

    public default void resetEncoders(){}

}
