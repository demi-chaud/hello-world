package driving;

import repast.simphony.annotate.AgentAnnot;
import repast.simphony.context.Context;
import repast.simphony.engine.schedule.ScheduledMethod;
import repast.simphony.util.ContextUtils;

/**
 * General agent class to handle creation, scheduling, and bookkeeping
 * @author Darryl Michaud
 */

@AgentAnnot(displayName = "Agent")
public class Agent {
	
	// Initialize variables
	//Parameters params = RunEnvironment.getInstance().getParameters();
	//double vehRho = (double)params.getValue("veh_vol");
	//double pedRho = (double)params.getValue("ped_vol");
	//double vLimit = (double)params.getValue("v_limit");
	//double timeStep = (double)params.getValue("time_step");
	//TODO: put these within step() (will need to change to @extend) to allow updates on the fly
	
	/**
	 * Method to schedule agent actions, details are overridden by subclasses
	 */
	@ScheduledMethod(start = 1, interval = 1)
	public void step() {
		//overridden
	}
	
    /**
     * Eliminates agents (called when agents move outside the space)
	*/
	@SuppressWarnings("rawtypes")
	public void die(){
		if (this.isCar() == 1) {
			Scheduler.allCars.remove(this);}
		else if (this.isPed() == 1) {
			Scheduler.allPeds.remove(this);}
		Context context = ContextUtils.getContext(this);
		context.remove(this);}

	/**
	 * Bookkeeping methods
	 * @return
	 */
	public int isCar() {
		return 0;}
	public int isPed() {
		return 0;}
	//public double getV() {
	//	return v;}
	//public void setV(double vel) {
	//	this.v = vel;}
}
