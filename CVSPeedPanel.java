import java.awt.BorderLayout;
import java.awt.Dimension;

import javax.swing.JPanel;

public class CVSPeedPanel extends JPanel{		
		private static final long serialVersionUID = -5112253105434137359L;

		CVSPeedPanel(roadPanel r)
		{
			super(new BorderLayout());
	    	this.setPreferredSize(new Dimension(650, 650));    	
	    	this.add(r, BorderLayout.CENTER);  
		}
	}