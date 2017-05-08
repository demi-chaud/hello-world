package driving1;

public class RedLight {
	public int timeInState;
	public state myState;
	public double xLoc;
	
	public static int amberT(double slKph) {
		double kphTOfps = 0.911344;
		double slFps = slKph * kphTOfps;
		double outTimeD;
		int	   outTime;
		outTimeD = 1 + slFps/20;
		outTime = (int)Math.round(outTimeD*10)/10;
		return outTime;
	}
	
	public enum state {
		GREEN, AMBER, RED;}
}
