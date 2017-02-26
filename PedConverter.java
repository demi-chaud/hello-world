package driving1;

import repast.simphony.parameter.StringConverter;

public class PedConverter implements StringConverter<Ped>{

	@Override
	public String toString(Ped obj) {
		return Integer.toHexString(obj.hashCode());}

	@Override
	public Ped fromString(String strRep) {
		return null;}
}
