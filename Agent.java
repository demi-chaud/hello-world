package driving1;

import repast.simphony.annotate.AgentAnnot;
import repast.simphony.context.Context;
import repast.simphony.engine.schedule.ScheduleParameters;
import repast.simphony.engine.schedule.ScheduledMethod;
import repast.simphony.util.ContextUtils;

/**
 * General agent class to handle creation, scheduling, and bookkeeping
 * @author Darryl Michaud
 */

@AgentAnnot(displayName = "Agent")
public class Agent {

	/**
	 * Method to schedule all agent observations and actions. 
	 * Details are overridden by subclasses
	 */
	@ScheduledMethod(start = 1, interval = 1, priority = 1)
	public void calc() {
		//overridden		
	}
	//TODO: maybe rewrite this as a for (i : allCars) to get it to behave
	
	/**
	 * Moves agents - scheduled after all other actions
	 */
	@ScheduledMethod(start = 1, interval = 1, priority = ScheduleParameters.LAST_PRIORITY)
	public void drive() {
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
