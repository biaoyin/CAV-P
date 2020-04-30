import java.util.ArrayList;

public class pedestrianDelay {

	double G_pedmin = Parameter.G_pedmin;
	double Tpc = Parameter.Tpc;
	double lamda1 ;
	double lamda2 ;
	double lamda3 ;
	double lamda4 ;
	
    double delay=0.0;
    double Tb = 0.0;
    int NS=0;
    int NSx = 0;
	ArrayList<Double> redDurationsApp1 = new ArrayList<Double>();
	ArrayList<Double> redDurationsApp2 = new ArrayList<Double>();
	

	public pedestrianDelay() {
		
	}
	
	pedestrianDelay (signalLight sig, double fP1, double fP2, double fP3, double fP4, double[] tax, double ta,  double tb, double lastTb, double tfdw, double tw_last, int Ns, ArrayList<Double> pedestrianApp1, ArrayList<Double> pedestrianApp2,ArrayList<Double> pedestrianApp3, ArrayList<Double> pedestrianApp4, ArrayList<Double> sigstartTimePed1, ArrayList<Double> sigstartTimePed2)
	{
		// variables generated
		double tw_next; 

		boolean PC; // judge pedestrian arrivals in PC interval or not
		NS = Ns;
		NSx= Ns; // number of switches considering extension
		
		lamda1 = fP1;
		lamda2 = fP2;		
		lamda3 = fP3;
		lamda4 = fP4;
		
		if (sig.phase==2 && (pedestrianApp1.isEmpty()==false || pedestrianApp3.isEmpty()==false)) 
			PC = true;	 
		else if (sig.phase==1 && (pedestrianApp2.isEmpty()==false || pedestrianApp4.isEmpty()==false))
			PC = true;
		else PC = false;

		tw_next = Math.max(tb + Tpc, tw_last + G_pedmin + Tpc);// the first next Walk switch time on any approach
		if (NS==0 && ta>=tfdw && tfdw>=tw_last + G_pedmin)
		   tw_next = tfdw + Tpc;//
		
		
		if (pedestrianApp1.isEmpty() && pedestrianApp2.isEmpty() && pedestrianApp3.isEmpty() && pedestrianApp4.isEmpty()) 
		{
			Tb = tb;
			//Tb = Math.max(tb, lastTb); //this is vehicle event, and lastTb is related to the previous pedestrian event ending time:  but this setting has not good results.
		}
		else if (PC == true) 
		{
			if (NS==0) {
				Tb = tw_next + 2*G_pedmin + Tpc;
				NSx = NSx + 2;
				
				if (sig.phase==2) {
					sigstartTimePed2.add(tw_next);//sigstartTimePed1 and sigstartTimePed2 restore the extension switch time. 
					sigstartTimePed1.add(tw_next+G_pedmin+Tpc);
				}
				else
				{
					sigstartTimePed1.add(tw_next);
					sigstartTimePed2.add(tw_next+G_pedmin+Tpc);
				}
			}
			
			else if (NS==1) {
				Tb = tw_next + G_pedmin;
				NSx ++;
				if (sig.phase==2) 
					sigstartTimePed1.add(tw_next);
				else
					sigstartTimePed2.add(tw_next);	
			}				 	
			else if (NS==2)
				Tb = Math.max(tb, tw_last + G_pedmin);
			else
				Tb=tb;
		}
		else if (PC == false)
		{
			if(NS==0) {//m=2
				
				Tb = tw_next + G_pedmin;
				NSx ++;
				if (sig.phase==2) 
					sigstartTimePed2.add(tw_next);
				else
					sigstartTimePed1.add(tw_next);		
			}
			else if(NS==1)
			{
				Tb = Math.max(tb, tw_last + G_pedmin);
			}				
			else
				Tb = tb;
		}	

		
		// calculate durations
		if (sig.phase==2) 
			redDurations(sig, ta,  tw_next,  Tb,  sigstartTimePed1, sigstartTimePed2);// reference solution- traffic green for approach 2, pedestrian green for approach 1.
		else
			redDurations(sig, ta,  tw_next,  Tb,  sigstartTimePed2, sigstartTimePed1); //for symmetric solution
		
		// calculate delay
		for (int i=0; i<redDurationsApp1.size(); i++) {
			delay+= 2*lamda1*Math.pow(redDurationsApp1.get(i),2)/2; // for two sides.
			delay+= 2*lamda3*Math.pow(redDurationsApp1.get(i),2)/2;			
		}
		if (sig.phase==1)
		{
			delay+= 2*lamda1*(ta-tax[0])*redDurationsApp1.get(0);  // only one side valid for approach 1, but estimate extra interval for two sides
			delay+= 2*lamda3*(ta-tax[2])*redDurationsApp1.get(0);  // only one side valid for approach 3, but estimate extra interval for two sides
			delay+= 2*lamda1*Math.pow((ta-tax[0]),2)/2;// add the extra "blank triangular"
			delay+= 2*lamda3*Math.pow((ta-tax[2]),2)/2;
		}
				
		for (int i=0; i<redDurationsApp2.size(); i++) {
			delay+= 2*lamda2*Math.pow(redDurationsApp2.get(i),2)/2;
			delay+= 2*lamda4*Math.pow(redDurationsApp2.get(i),2)/2;
		}
		if (sig.phase==2)
		{
			delay+= 2*lamda2*(ta-tax[1])*redDurationsApp2.get(0);  // only one side valid for approach 2, but estimate extra interval for two sides
			delay+= 2*lamda4*(ta-tax[3])*redDurationsApp2.get(0);  // only one side valid for approach 4, but estimate extra interval for two sides
			delay+= 2*lamda2*Math.pow((ta-tax[1]),2)/2;
			delay+= 2*lamda4*Math.pow((ta-tax[3]),2)/2;
		}
	}
	
	void  redDurations(signalLight sig, double ta, double tw_next, double Tb, ArrayList<Double> sigstartTimePed1, ArrayList<Double> sigstartTimePed2)
	{
	    // reference: traffic green for approach 2, pedestrian green for approach 1.	
		double tf=0;
		double dur=0;

		ArrayList<Double> redDurationsapp1 = new ArrayList<Double> ();
		ArrayList<Double> redDurationsapp2 = new ArrayList<Double> ();

		// for waiting pedestrian at approach 1, assume that traffic green for approach 2
		if (sigstartTimePed1.isEmpty()) // only NSx=0 or 1
		{
			if (NSx==0) 
			{						
				tf = tw_next-Tpc;
				dur = Math.max(0, Tb - tf);	
				redDurationsapp1.add(dur);					
			}
			else
			{
				tf = sigstartTimePed2.get(0)-Tpc;
				dur = Math.max(0, Tb - tf);		
				redDurationsapp1.add(dur);	
			}

		}
		else //NSx>1
		{
			tf= sigstartTimePed2.get(0)-Tpc;
			dur = sigstartTimePed1.get(0) - tf;	
			redDurationsapp1.add(dur); // first triangular
			if (sigstartTimePed1.size()>1)
			{
				for (int i=1; i < sigstartTimePed1.size(); i++)
				{
					tf = sigstartTimePed2.get(i-1)-Tpc;
					dur = sigstartTimePed1.get(i) - tf;
					redDurationsapp1.add(dur);
				}
				
				if (NSx % 2 ==1) // the last triangular
				{
					tf = sigstartTimePed2.get(sigstartTimePed2.size()-1) - Tpc;
					dur = Tb - tf;
					redDurationsapp1.add(dur);
				}
			}
			
		}

		// for passing pedestrian at approach 2
		if (sigstartTimePed2.isEmpty()) // only NSx =0;
		{		
			tf = ta;
			dur = Math.max(0, Tb - tf);		
			redDurationsapp2.add(dur);					
		}
		else // NSx>0
		{
			tf = ta;
			dur = sigstartTimePed2.get(0) - tf;	
			redDurationsapp2.add(dur); // first triangular
			if (sigstartTimePed2.size()>1)
			{
				for (int i=1; i < sigstartTimePed2.size(); i++)
				{
					tf = sigstartTimePed1.get(i-1)-Tpc;
					dur = sigstartTimePed2.get(i) - tf;
					redDurationsapp2.add(dur);
				}
				
				if (NSx % 2 == 0)
				{
					tf = sigstartTimePed1.get(sigstartTimePed1.size()-1) - Tpc;
					dur = Tb - tf;
					redDurationsapp2.add(dur);
				}
			}
	
		}
		
		//
		double para=0.2;//it is a calibrated coefficient
		if (sig.phase==2) //traffic green for approach 2, pedestrian green for approach 1.
		{
			for (int i=0; i<redDurationsapp1.size(); i++)
				redDurationsApp1.add(redDurationsapp1.get(i)-para*Tpc);
			for (int i=0; i<redDurationsapp2.size(); i++)
				redDurationsApp2.add(redDurationsapp2.get(i)-para*Tpc);
		}
		else  // symmetric solution
		{
			for (int i=0; i<redDurationsapp1.size(); i++)
				redDurationsApp2.add(redDurationsapp1.get(i)-para*Tpc);
			for (int i=0; i<redDurationsapp2.size(); i++)
				redDurationsApp1.add(redDurationsapp2.get(i)-para*Tpc);
		}

	}
	

	
}
