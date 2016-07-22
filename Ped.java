package driving;

import repast.simphony.space.continuous.ContinuousSpace;
import repast.simphony.space.continuous.NdPoint;
import repast.simphony.space.grid.Grid;

/**
 * Pedestrian agent class
 * @author yawei
 */

public class Ped extends Agent{
	private ContinuousSpace<Object> space;
	private Grid<Object> grid;
	private double v, v0, walkage;
	private int dir;  // 1 walks up, 0 walks down
	
	
	@Override 
	public void step() {
		walk();
	}
	
	public void walk() {
		NdPoint myLoc = space.getLocation(this);
		if (dir == 1) {walkage = v; } 
			else {walkage = -v; }
		space.moveByDisplacement(this,0,walkage);
		myLoc = space.getLocation(this);
		grid.moveTo(this,(int)myLoc.getX(),(int)myLoc.getY());
	}	
	
	public Ped(ContinuousSpace<Object> space, Grid<Object> grid, double vkm) {
		this.space = space;
		this.grid  = grid;
		v0 = (double)vkm * 1000 / 3600;
		this.v     = v0;
	}
	
	/**
	 * Getter for identification
	 */
	@Override
	public int isPed() {
		return 1;}
}
