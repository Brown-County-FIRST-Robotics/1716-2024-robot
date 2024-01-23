package frc.robot.utils;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.inputs.LoggedDriverStation;
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;
import org.littletonrobotics.junction.inputs.LoggedSystemStats;

import java.util.ArrayList;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class CustomAlerts {
  public static class CustomAlert extends PeriodicRunnable{
    Alert alert;
    BooleanSupplier isActive;
    Supplier<String> message;

    public CustomAlert(Alert.AlertType level, BooleanSupplier isActive, Supplier<String> message) {
      super();
      this.isActive = isActive;
      this.message = message;
      alert=new Alert(message.get(),level);
    }
    public CustomAlert(Alert.AlertType level, BooleanSupplier isActive, String message){
      this(level,isActive,()-> message);
    }

    @Override
    public void periodic() {
      if(isActive.getAsBoolean()){
        alert.setText(message.get());
        alert.set(true);
      }else {
        alert.set(false);
      }
    }
  }
  public static class TimeoutAlert extends PeriodicRunnable{
    final double timeout;
    Alert alert;
    double lastFeedTime;

    public TimeoutAlert(Alert.AlertType level, double timeout, String message) {
      super();
      this.timeout = timeout;
      lastFeedTime= Timer.getFPGATimestamp();
      alert=new Alert(message,level);
    }
    public void feed(){
      lastFeedTime= Timer.getFPGATimestamp();
    }

    @Override
    public void periodic() {
      alert.set(Timer.getFPGATimestamp()>lastFeedTime+timeout);
    }
  }
  public static void makeOverTempAlert(CANSparkBase spark, double errTemp, String deviceName){
    new CustomAlert(Alert.AlertType.ERROR, ()-> spark.getMotorTemperature()>errTemp,()-> deviceName+" is currently "+spark.getMotorTemperature()+" degrees celsius (max " +errTemp+"). Prolonged usage could permanently damage the motor");
  }

  public static void makeOverTempAlert(CANSparkBase spark, double errTemp, double warnTemp, String deviceName){
    makeOverTempAlert(spark,errTemp,deviceName);
    new CustomAlert(Alert.AlertType.WARNING, ()-> (spark.getMotorTemperature()>warnTemp && spark.getMotorTemperature()<errTemp),()-> deviceName+" is currently "+spark.getMotorTemperature()+" degrees celsius (max \" +errTemp+\")");
  }
  public static void makeCANFailAlerts(double errUtilization){
    new CustomAlert(Alert.AlertType.ERROR, ()-> errUtilization<LoggedSystemStats.getInputs().canStatus.percentBusUtilization, ()-> "CAN Bus utilization is "+LoggedSystemStats.getInputs().canStatus.percentBusUtilization+"(max "+errUtilization+"). All mechanisms may soon cease to function. ");
  }
  public static void makeNavxFailAlerts(AHRS navx){
    new CustomAlert(Alert.AlertType.ERROR, ()-> !navx.isConnected(),"Navx is not connected. This will cause pose estimation and field-oriented driving to fail. ");
    new CustomAlert(Alert.AlertType.WARNING, navx::isCalibrating,"Navx is calibrating. Please do not move the robot. ");
  }

}
