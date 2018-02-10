package driving1;

import repast.simphony.annotate.AgentAnnot;
import repast.simphony.context.Context;
import repast.simphony.space.continuous.ContinuousSpace;
import repast.simphony.space.continuous.NdPoint;
import repast.simphony.util.ContextUtils;

/**
 * General agent class for bookkeeping
 * @author Darryl Michaud
 */

@AgentAnnot(displayName = "Agent")
public class Agent {
	ContinuousSpace<Object> space;
	
    /**
     * Eliminates agents (called when agents move outside the space)
	 */
	@SuppressWarnings("rawtypes")
	public void die(){
		if (this.isCar()) {
			Scheduler.allCars.remove(this);}
		else if (this.isPed()) {
			Scheduler.allPeds.remove(this);}
		Context context = ContextUtils.getContext(this);
		context.remove(this);
	}

	/**
	 * Bookkeeping methods
	 * @return
	 */
	public boolean isCar() {
		return false;}
	public boolean isPed() {
		return false;}
}
