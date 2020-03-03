public class PI {
  private PIParameters p;

  private double I; // Integrator state
  
  private double v; // Desired control signal
  private double e; // Current control error
  
  //Constructor
  public PI(String name) {
	  PIParameters p = new PIParameters();
	  p.Beta = 1.0;
	  p.H = 0.04;
	  p.integratorOn = false;
	  p.K = 3;
	  p.Ti = 0.0;
	  p.Tr = 10.0;
	  //new PIGUI(this, p, name);
	  setParameters(p);
	  
	  I = 0.0;
	  v = 0.0;
	  e = 0.0;
  }

  public synchronized double calculateOutput(double y, double yref) {
	  e = yref - y;
	  v = p.K * (p.Beta * yref - y) + I; // I is 0.0 if integratorOn is false
	  return v;
  }

  public synchronized void updateState(double u) {
	  if (p.integratorOn) {
		  I = I + (p.K * p.H / p.Ti) * e + (p.H / p.Tr) * (u - v);
	  } else {
		  I = 0.0;
	  }
  }
	
  public synchronized long getHMillis() {
  	return (long)(p.H * 1000.0);
  }

  public synchronized void setParameters(PIParameters newParameters) {
  	p = (PIParameters)newParameters.clone();
	if (!p.integratorOn) {
		I = 0.0;
	}
  }
  
  // Sets the I-part of the controller to 0.
  // For example needed when changing controller mode.
  public synchronized void reset() {
	  I = 0.0;
  }
  
  // Returns the current PIParameters.
  public synchronized PIParameters getParameters() {
	  return p;
  }
}
   