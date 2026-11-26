package util;

public class PIDFController {
    private double kP, kI, kD, kF;

    private double setpoint=0;
    private double integral=0;
    private double lastError=0;
    private boolean firstRun=true;

    public PIDFController(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }

    public void setCoefficients(double kP, double kI, double kD, double kF){
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }

    public void setSetpoint(double setpoint){
        this.setpoint = setpoint;
    }

    public void reset(){
        integral=0;
        lastError=0;
        firstRun=true;
    }

   public double update (double current, double dt, double ffInput)
   {
       double error = setpoint - current;

       if (firstRun) {
           lastError = error;
           firstRun = false;
       }

       integral += error * dt;

       double derivative = (error - lastError) / dt;
       lastError = error;

       double pTerm=kP*error;
       double iTerm=kI*integral;
       double dTerm=kD*derivative;
       double fTerm=kF*ffInput;

       return pTerm+iTerm+dTerm+fTerm;
   }

}
