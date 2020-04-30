import java.awt.BasicStroke;
import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.RenderingHints;

import javax.swing.JPanel;

public class roadPanel extends JPanel{ //visualization
		/**
		 * 
		 */
	double zoneLength = Parameter.zoneLength; // communication range
	double loopLocation = Parameter.loopLocation; // loop for actuated control
	double lenIS = Parameter.lenIS;
	
	private static final long serialVersionUID = 6281777298606990928L;
	// forward
	Lane lane1; //E
	Lane lane2; //S
	Lane lane3;// W
	Lane lane4;// N
	
	signalLight signal;
	public roadPanel(Lane l1, Lane l2, Lane l3, Lane l4, signalLight s) {
		super(new BorderLayout());
		this.setPreferredSize(new Dimension(640, 640));
		this.setBackground(Color.white); 
		lane1 = l1;
		lane2 = l2;
		lane3 = l3;
		lane4 = l4;

		signal = s;
	}

	protected void paintComponent(Graphics g)
	{
		super.paintComponent(g);
		Graphics2D g2d = (Graphics2D) g;
		g2d.setRenderingHint(
				RenderingHints.KEY_ANTIALIASING,
				RenderingHints.VALUE_ANTIALIAS_ON);
		
		int pixel=3; // original set is 5;
		int lenApp = (int) ((zoneLength+16)*pixel);
		int widLane = 5*pixel;
		int widInter = 2 * widLane;
		
		// draw lanes
		g2d.setColor(Color.lightGray);
		g2d.fillRect(0, lenApp, lenApp*2+widInter, widLane);// E--W
		g2d.fillRect(0, lenApp + widLane, lenApp*2+widInter, widLane);//W-E
		g2d.fillRect(lenApp + widLane, 0, widLane, lenApp*2+widInter); // S-N
		g2d.fillRect(lenApp, 0, widLane , lenApp*2+widInter); // N-S
		
		// draw cars
		for(Car cp: lane1.carList)
		{    	    	
			if(cp.location>=-100 && cp.location<=zoneLength+lenIS)//E
			{
				g2d.setColor(cp.color);
				g.fillRect((int) (-cp.location*pixel)+lenApp+widInter, lenApp+pixel, cp.w*pixel, cp.h*pixel);
			}    	    	
		}

		for(Car cp: lane2.carList)//S
		{    	    	
			if(cp.location>=-100 && cp.location<=zoneLength+lenIS)
			{
				g2d.setColor(cp.color);
				g.fillRect(lenApp+widLane+pixel, (int) (-cp.location*pixel)+lenApp+widInter, cp.h*pixel, cp.w*pixel);
			}
		}
		
		for(Car cp: lane3.carList)//W
		{    	    	
			if(cp.location>=-100 && cp.location<=zoneLength+lenIS)
			{
				g2d.setColor(cp.color);
				g.fillRect((int) (lenApp+(cp.location-cp.length)*pixel), lenApp+widLane+pixel, cp.w*pixel, cp.h*pixel);
			}    	    	
		}
		
		for(Car cp: lane4.carList)//N
		{    	    	
			if(cp.location>=-100 && cp.location<=zoneLength+lenIS)
			{
				g2d.setColor(cp.color);
				g.fillRect(lenApp+pixel, (int) (lenApp+(cp.location-cp.length)*pixel), cp.h*pixel, cp.w*pixel);
			}    	    	
		}


		float [] dash1 = {10.0f};
		g2d.setColor(Color.black);
		g2d.setStroke(new BasicStroke(1.0f, BasicStroke.CAP_BUTT, BasicStroke.JOIN_MITER, 1.0f,dash1 , 0.0f));
		// draw arrival line
		g2d.drawLine(lenApp+widInter+pixel*(int)zoneLength, lenApp+3*widLane, lenApp+widInter+pixel*(int)zoneLength, lenApp-widLane);//E
		g2d.drawLine(lenApp+3*widLane, lenApp+widInter+pixel*(int)zoneLength, lenApp-widLane, lenApp+widInter+pixel*(int)zoneLength);//S
		g2d.drawLine(pixel*16, lenApp+3*widLane, pixel*16, lenApp-widLane);//W
		g2d.drawLine(lenApp+3*widLane, pixel*16, lenApp-widLane, pixel*16);//N
		// draw  loop
		g.drawRect((int) (-loopLocation*pixel)+lenApp+widInter, lenApp+pixel, widLane-2*pixel, widLane-2*pixel);//E
		g.drawRect(lenApp+widLane+pixel, (int) (-loopLocation*pixel)+lenApp+widInter, widLane-2*pixel, widLane-2*pixel);//S		
		g.drawRect(lenApp- (int) (-loopLocation*pixel)-(widLane-2*pixel), lenApp+widLane+pixel, widLane-2*pixel, widLane-2*pixel); //W
		g.drawRect(lenApp+pixel,lenApp- (int) (-loopLocation*pixel)-(widLane-2*pixel), widLane-2*pixel, widLane-2*pixel); //N

		g2d.setStroke(new BasicStroke(3.0f));
		// draw lights
		switch(signal.phase)
		{
		case 1:
			g2d.setColor(Color.green);
			g2d.drawLine(lenApp+widInter, lenApp, lenApp+widInter, lenApp+widLane);//E
			g2d.drawLine(lenApp, lenApp+widLane, lenApp, lenApp+2*widLane);//W
			g2d.setColor(Color.red);
			g2d.drawLine(lenApp+widLane, lenApp+widInter, lenApp+2*widLane, lenApp+widInter);//S
			g2d.drawLine(lenApp, lenApp, lenApp+widLane, lenApp);//N
	
			break;

		case 2:
			g2d.setColor(Color.green);
			g2d.drawLine(lenApp+widLane, lenApp+widInter, lenApp+2*widLane, lenApp+widInter);//S
			g2d.drawLine(lenApp, lenApp, lenApp+widLane, lenApp);//N
			g2d.setColor(Color.red);
			g2d.drawLine(lenApp+widInter, lenApp, lenApp+widInter, lenApp+widLane);//E
			g2d.drawLine(lenApp, lenApp+widLane, lenApp, lenApp+2*widLane);//W

			break;
		}
		
		

		this.setLayout(new BorderLayout());

	}
	}