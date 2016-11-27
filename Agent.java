package driving1;

import repast.simphony.annotate.AgentAnnot;
import repast.simphony.context.Context;
import repast.simphony.util.ContextUtils;

/**
 * General agent class for bookkeeping
 * @author Darryl Michaud
 */

@AgentAnnot(displayName = "Agent")
public class Agent {
	
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
		context.remove(this);
	}

	/**
	 * Bookkeeping methods
	 * @return
	 */
	public int isCar() {
		return 0;}
	public int isPed() {
		return 0;}
}
