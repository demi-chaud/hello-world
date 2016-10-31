package driving1;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.*;

import repast.simphony.ui.RSApplication;

public class UserPanel implements ActionListener{

	static public boolean carsYes = false;  // turns cars on/off for testing
	static public boolean pedsYes = true;   // ditto for peds
	static public int vehRho = 600;         // veh/hr
	static public int pedRho = 60;          // ppl/hr
	private String vehRhos = String.valueOf(vehRho);
	private String pedRhos = String.valueOf(pedRho);
	
	
	public UserPanel() {
		JPanel newPanel = new JPanel();
		JCheckBox vehOn = new JCheckBox("Vehicles?", false);    //TODO: switch this default once it all works
		JCheckBox pedOn = new JCheckBox("Pedestrians?", true); 
		JLabel   vLabel = new JLabel("Vehicles per hour:");
		JTextField vRho = new JTextField(vehRhos, 4);
			vRho.setActionCommand("vRho");
		JLabel   pLabel = new JLabel("Pedestrians per hour:");
		JTextField pRho = new JTextField(pedRhos, 4);
			pRho.setActionCommand("pRho");
		
		vehOn.addActionListener(this);
		pedOn.addActionListener(this);
		pRho.addActionListener(this);
		vRho.addActionListener(this);
		
		newPanel.add(vehOn);
		newPanel.add(pedOn);
		newPanel.add(vLabel);
		newPanel.add(vRho);
		newPanel.add(pLabel);
		newPanel.add(pRho);
				
		newPanel.setSize(50,100);
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
			case "Vehicles?":
				if (checkSource.isSelected()) carsYes = true;
				else carsYes = false;
				break;
			case "Pedestrians?":
				if (checkSource.isSelected()) pedsYes = true;
				else pedsYes = false;
				break;
			default: break;}
			break;
		case "TextFieldUI":
			JTextField textSource = (JTextField)which;
			switch (name) {
			case "vRho":
				String newVVal = textSource.getText();
				vehRho = Integer.parseInt(newVVal);
				break;
			case "pRho":
				String newPVal = textSource.getText();
				pedRho = Integer.parseInt(newPVal);
				break;}
			break;
		default: break;}
	}
}
