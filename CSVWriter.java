package driving1;

import java.io.FileWriter;
import java.io.IOException;
import java.text.DecimalFormat;
import java.text.DecimalFormatSymbols;
import java.text.NumberFormat;
import java.util.ArrayList;
import java.util.Locale;

public class CSVWriter {
	
	public static void writeCSV(String fileName, ArrayList<Turtle.Conflict> list) {
		String cString, turtle, dirC, lane, ped, dirP, ying, vel, dec, ttc, init;
		String comma  = ",";
		String nLine  = "\n";
		String header = "turtle,dirC,lane,ped,dirP,ying,vel,dec,ttc,init" + nLine;
		FileWriter fileWriter = null;
		try {
			fileWriter = new FileWriter(fileName);
			fileWriter.append(header);
			NumberFormat form = new DecimalFormat("#.0####",
					DecimalFormatSymbols.getInstance(Locale.ENGLISH));
			for (Turtle.Conflict c : list) {
				String turtle0	= Integer.toHexString(c.car.hashCode());
				String dirC0	= String.valueOf(c.dirC);
				String lane0	= String.valueOf(c.lane);
				String ped0		= Integer.toHexString(c.ped.hashCode());
				String dirP0	= String.valueOf(c.dirP);
				String ying0	= String.valueOf(c.ying);
				Double vKH		= c.vel*UserPanel.vBase;
				String v0		= form.format(vKH);
				Double decMS	= c.yDec*UserPanel.spaceScale/(UserPanel.tStep*UserPanel.tStep);
				String dec0		= form.format(decMS);
				Double ttcS		= c.TTC*UserPanel.tStep;
				String ttc0		= form.format(ttcS);
				String init0	= String.valueOf(c.init);
				turtle	= turtle0 + comma;
				dirC	= dirC0 + comma;
				lane	= lane0 + comma;
				ped		= ped0 + comma;
				dirP	= dirP0 + comma;
				ying	= ying0 + comma;
				vel		= v0 + comma;
				dec		= dec0 + comma;
				ttc		= ttc0 + comma;
				init	= init0 + comma;
				cString = turtle + dirC + lane + ped + dirP + ying + vel + dec + ttc + init + nLine;
				fileWriter.append(cString);}}
		catch (IOException e) {
			System.out.print("Error in CSVWriter");
			e.printStackTrace();}
		finally {
			try {
				fileWriter.flush();
				fileWriter.close();}
			catch (IOException e1) {
				System.out.print("Error flushing CSVWriter");}}}
}
