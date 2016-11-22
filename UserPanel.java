package driving1;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.*;

import repast.simphony.ui.RSApplication;

public class UserPanel implements ActionListener{

//	static public boolean carsYes = true;		// turns cars on/off for testing
//	static public boolean pedsYes = false;		// ditto for peds
//	static public boolean reactDelay = true;	// delayed reactions
	static public boolean estErr = true;		// estimation errors
	static public double  sLimit = 45;			// km/hr		test: 45
	static public int     vehRho = 600;			// veh/hr       test: 600
	static public int     pedRho = 0;			// ppl/hr       test: 60
	static public double  delayT = 0.5;			// seconds      test: .5
	static public double  tStep  = 0.18;		// seconds
	static public double  lambdaCar = vehRho * tStep / 3600;	// these four are recalculated here on edits
	static public double  lambdaPed = pedRho * tStep / 3600;
	static public double  poisExpV  = Math.exp(-lambdaCar);
	static public double  poisExpP  = Math.exp(-lambdaPed);
	static public double  Pof1Car = lambdaCar * poisExpV; // Poisson probability of 1 arrival in a tick (higher counts are negligible)
	static public double  Pof1Ped = lambdaPed * poisExpP; // ditto for peds
	
	private String sLimits = String.valueOf(sLimit);
	private String vehRhos = String.valueOf(vehRho);
	private String pedRhos = String.valueOf(pedRho);
	private String delTs   = String.valueOf(delayT);
	private String tSteps  = String.valueOf(tStep);
	
	
	public UserPanel() {
		JPanel newPanel = new JPanel();
//		JCheckBox vehOn = new JCheckBox("Vehicles?", true);    
//		JCheckBox pedOn = new JCheckBox("Pedestrians?", true);
//		JCheckBox delOn = new JCheckBox("Delayed reactions?", true);
		JCheckBox errOn = new JCheckBox("Estimation Errors?", true);
		
		JLabel   sLabel = new JLabel("Speed limit in km/hr:");
		JTextField sLim = new JTextField(sLimits, 4);
			sLim.setActionCommand("sLim");
		JLabel   vLabel = new JLabel("Vehicles per hour:");
		JTextField vRho = new JTextField(vehRhos, 4);
			vRho.setActionCommand("vRho");
		JLabel   pLabel = new JLabel("Pedestrians per hour:");
		JTextField pRho = new JTextField(pedRhos, 4);
			pRho.setActionCommand("pRho");
		JLabel   tLabel = new JLabel("Reaction time in sec:");
		JTextField delT = new JTextField(delTs,   6);
			delT.setActionCommand("delT");
		JLabel   tckLab = new JLabel("Tick duration in sec:");
		JTextField tckT = new JTextField(tSteps,  5);
			tckT.setActionCommand("tckT");
			
//		vehOn.addActionListener(this);
//		pedOn.addActionListener(this);
//		delOn.addActionListener(this);
		errOn.addActionListener(this);
		sLim.addActionListener(this);
		pRho.addActionListener(this);
		vRho.addActionListener(this);
		delT.addActionListener(this);
		tckT.addActionListener(this);
		
//		newPanel.add(vehOn);
//		newPanel.add(pedOn);
//		newPanel.add(delOn);
		newPanel.add(errOn);
		newPanel.add(sLabel);
		newPanel.add(sLim);
		newPanel.add(vLabel);
		newPanel.add(vRho);
		newPanel.add(pLabel);
		newPanel.add(pRho);
		newPanel.add(tLabel);
		newPanel.add(delT);
		newPanel.add(tckLab);
		newPanel.add(tckT);
				
//		newPanel.setSize(50,100);
		RSApplication.getRSApplicationInstance().addCustomUserPanel(newPanel);	
	}

	public void actionPerformed(ActionEvent ae) {
		JComponent which = (JComponent)ae.getSource();
		String type = which.getUIClassID();
		String name = ae.getActionCommand();
		switch (type) {
		case "CheckBoxUI":
			JCheckBox checkSource = (JCheckBox)which;
			switch (name) {
//			case "Vehicles?":
//				if (checkSource.isSelected()) carsYes = true;
//				else carsYes = false;
//				break;
//			case "Pedestrians?":
//				if (checkSource.isSelected()) pedsYes = true;
//				else pedsYes = false;
//				break;
//			case "Delayed reactions?":
//				if (checkSource.isSelected()) reactDelay = true;
//				else reactDelay = false;
//				break;
			case "Estimation Errors?":
				if (checkSource.isSelected()) estErr = true;
				else estErr = false;
				break;
			default: break;}
			break;
		case "TextFieldUI":
			JTextField textSource = (JTextField)which;
			switch (name) {
			case "sLim":
				String newSVal = textSource.getText();
				sLimit = Integer.parseInt(newSVal);
				break;
			case "vRho":
				String newVVal = textSource.getText();
				vehRho = Integer.parseInt(newVVal);
				calcCars();
				break;
			case "pRho":
				String newPVal = textSource.getText();
				pedRho = Integer.parseInt(newPVal);
				calcPeds();
				break;
			case "delT":
				String newTVal = textSource.getText();
				delayT = Double.parseDouble(newTVal);
				break;
			case "tckT":
				String newTick = textSource.getText();
				tStep = Double.parseDouble(newTick);
				calcCars();
				calcPeds();
				break;}
			break;
		default: break;}
	}
	
	/**
	 * Calculates new values for car creation
	 */
	public void calcCars(){
		lambdaCar = vehRho * tStep / 3600;		// veh arrivals per tick
		poisExpV  = Math.exp(-lambdaCar);
		Pof1Car = lambdaCar * poisExpV;
	}
	
	/**
	 * Calculates new values for ped creation
	 */
	public void calcPeds(){
		lambdaPed = pedRho * tStep / 3600;		// ped arrivals per tick
		poisExpP  = Math.exp(-lambdaPed);
		Pof1Ped = lambdaPed * poisExpP;
	}
}


/*int poisTerms = 5;
double poisThisP, poisThisV;
double[] poisStoreV = new double[poisTerms];
double[] poisStoreP = new double[poisTerms];
int i;
for (i=0;i<poisTerms;i++) {  //TODO: 5 is arbitrary here, make more general by setting the size of result as loop stop criterion
									//can switch to vector<> and .add to make it grow
	poisStoreV[i] = Math.pow(lambdaCar,i+1)*poisExpV/factorial(i+1);
	poisStoreP[i] = Math.pow(lambdaPed,i+1)*poisExpP/factorial(i+1);}*/

