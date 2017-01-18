package driving1;

import repast.simphony.parameter.StringConverter;

public class turtleConverter implements StringConverter<Turtle>{

	@Override
	public String toString(Turtle obj) {
		return Integer.toHexString(obj.hashCode());}

	@Override
	public Turtle fromString(String strRep) {
		return null;}
}
