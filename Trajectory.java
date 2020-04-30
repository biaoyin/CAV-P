
public class Trajectory {

	
	double amax = Parameter.amax; //maximum acceleration 0.73
	double a = amax;
	double gmax = Parameter.gmax; // maximum green duration(s)
	double gmin = Parameter.gmin; // minimum green duration(s)
	double vf = Parameter.vf; // speed
	double length = Parameter.length; // car length
	double lenIS = Parameter.lenIS;  // length of intersection
	double cap = Parameter.cap;
	double vmin = Parameter.vmin;	// minimum of cruise vehicle speed
    double Tar = Parameter.Tar;
    double Tpc = Parameter.Tpc;
	double deptime;
	double speed;
	double leadspeed;
	
	int NS = 0;
	double tw_last = 0;
	
	public Trajectory(){

	}	
	
	Trajectory (double time, double lastdeptime, signalLight sig, Car currentCar, int depapp,  int platoon,  double leadspeed0)
	{
		// assume signal change for delay estimate  
        tw_last = sig.startTime; // NS = 0;
		if(lastdeptime> sig.startTime+gmax && sig.phase == depapp) 
		{
			sig.switchSignal(lastdeptime);	
			NS++;
			sig.switchSignal(lastdeptime  + gmin);
			NS++;
			tw_last = sig.startTime;
		}
		else if(sig.phase != depapp)
		{
			
			if(sig.startTime + gmin > lastdeptime ) 
			{
				sig.switchSignal(sig.startTime + gmin);
				NS++;
				tw_last = sig.startTime;
			}
			else
			{
				sig.switchSignal(lastdeptime);
				NS++;
				tw_last = sig.startTime;
			}		    				
		}			
		
		if(!currentCar.equipped)
		{
			leadspeed = 0.0;
			double penalty = Math.max(lenIS/vf, (-(platoon-1)/cap*a+ Math.sqrt(Math.pow((platoon-1)/cap*a,2.0) + 2*a*lenIS))/a);
			deptime= Math.max(-currentCar.location/vf+lenIS/vf+time,Math.max(lastdeptime+1/cap+penalty, sig.startTime+penalty));
			speed= -1.0;
		}
		else if(!currentCar.automated)
		{
			if(currentCar.location/vf+time<= Math.max(lastdeptime+1/cap, sig.startTime)||currentCar.stop()) 
			{
				double penalty = Math.max(lenIS/vf, (-(platoon-1)/cap*a+ Math.sqrt(Math.pow((platoon-1)/cap*a,2.0) + 2*a*lenIS))/a);
				deptime = Math.max(-currentCar.location/vf+lenIS/vf+time,Math.max(lastdeptime+1/cap+penalty, sig.startTime+penalty));
				leadspeed = 0.0;
				speed = -1.0;
			}
			else
			{
				deptime = -currentCar.location/vf+lenIS/vf+time;
				leadspeed = vf;
				speed = -1.0;
			}
		}
		else
		{
			if(!currentCar.stop())
			{
				if(platoon==1)
				{
					double vnew = speed2(-currentCar.location, time, lastdeptime, sig.startTime);//optimal speed
					if(vnew > vf || vnew < 0)
					{
						vnew = vf;
						leadspeed = vf; //initial speed
						deptime = -currentCar.location/vf+lenIS/vf+time;
						speed = vf; // designed speed
					}
					else if(vnew > vmin)
					{
						double penalty = Math.max(lenIS/vf, (-vnew+ Math.sqrt(Math.pow(vnew,2.0) + 2*a*lenIS))/a);
						deptime = Math.max(-currentCar.location/vf+lenIS/vf+time, Math.max(lastdeptime+1/cap+penalty, sig.startTime+penalty));
						deptime = Math.max(lastdeptime+1/cap+penalty, sig.startTime+penalty);
						leadspeed = vnew;
						speed = vnew;
					}
					else
					{
						double penalty = Math.max(lenIS/vf, (Math.sqrt(Math.pow(vnew,2.0) + 2*a*lenIS))/a);
						deptime = Math.max(-currentCar.location/vf+lenIS/vf+time, Math.max(lastdeptime+1/cap+penalty, sig.startTime+penalty));
						leadspeed = 0;
						speed = vmin;
					}

				}
				else
				{
					if(leadspeed0>vf-1e-3)
					{
						deptime = Math.max(-currentCar.location/vf+time+lenIS/vf, Math.max(lastdeptime+1/cap+lenIS/vf, sig.startTime+lenIS/vf));//byin
						speed = -1.0;
					}
					else
					{
						double vnew = Math.min(leadspeed0+(platoon-1)*a/cap, vf);
						double penalty = Math.max(lenIS/vf, (-vnew + Math.sqrt(Math.pow(vnew,2.0) + 2*a*lenIS))/a);
						deptime = Math.max(-currentCar.location/vf+lenIS/vf+time,Math.max(lastdeptime+1/cap+penalty, sig.startTime+penalty));
						leadspeed = 0;
						speed = -1.0;
					}

				}
			}
			else
			{
				double vnew = Math.min(leadspeed0+(platoon-1)*a/cap, vf);
				double penalty = Math.max(lenIS/vf, (-vnew + Math.sqrt(Math.pow(vnew,2.0) + 2*a*lenIS))/a);
				deptime = Math.max(lastdeptime+1/cap+penalty, sig.startTime+penalty);
				leadspeed = 0;
				speed = -1.0;
			}

		}
	}
	
	
	// for B&B delay estimation: 
	Trajectory (double time, double lastdeptime, signalLight sig, Car currentCar, int depapp,  int platoon,  double leadspeed0,  boolean flag)
	{
		
		
		if(lastdeptime>sig.startTime+gmax && sig.phase == depapp) 
		{
			sig.switchSignal(lastdeptime);
			sig.switchSignal(lastdeptime+gmin);
		}
		else if(sig.phase != depapp)
		{
			if(sig.startTime + gmin > lastdeptime)
				sig.switchSignal(sig.startTime + gmin);
			else
				sig.switchSignal(lastdeptime);	    				
		}

		
		
		if(!currentCar.equipped)
		{
			leadspeed = 0.0;
			double penalty = Math.max(lenIS/vf, (-(platoon-1)/cap*a+ Math.sqrt(Math.pow((platoon-1)/cap*a,2.0) + 2*a*lenIS))/a);
			if (flag) // flag = ture is for ideal estimation
				deptime = Math.max(lastdeptime+1/cap+lenIS/vf, sig.startTime+lenIS/vf); //LB
			else // flag = false is for real estimation
				deptime = Math.max(-currentCar.location/vf+lenIS/vf+time,Math.max(lastdeptime+1/cap+penalty, sig.startTime+penalty)); //for UB
			speed= -1.0;
		}
		else if(!currentCar.automated)
		{
			if(currentCar.location/vf+time<= Math.max(lastdeptime+1/cap, sig.startTime)||currentCar.stop()) 
			{
				double penalty = Math.max(lenIS/vf, (-(platoon-1)/cap*a+ Math.sqrt(Math.pow((platoon-1)/cap*a,2.0) + 2*a*lenIS))/a);
				if (flag)
					deptime = Math.max(lastdeptime+1/cap+lenIS/vf, sig.startTime+lenIS/vf);
				else
					deptime = Math.max(-currentCar.location/vf+lenIS/vf+time,Math.max(lastdeptime+1/cap+penalty, sig.startTime+penalty));				
				leadspeed = 0.0;
				speed = -1.0;
			}
			else
			{
				if (flag)
					deptime = Math.max(lastdeptime+1/cap+lenIS/vf, sig.startTime+lenIS/vf);
				else		
					deptime = -currentCar.location/vf+lenIS/vf+time;
				leadspeed = vf;
				speed = -1.0;
			}
		}
		else
		{
			if(!currentCar.stop())
			{
				if(platoon==1)
				{
					double vnew = speed2(-currentCar.location, time, lastdeptime, sig.startTime);//optimal speed
					if(vnew > vf || vnew < 0)
					{
						vnew = vf;	
						if (flag)
							deptime = Math.max(lastdeptime+1/cap+lenIS/vf, sig.startTime+lenIS/vf);
						else		
							deptime = -currentCar.location/vf+lenIS/vf+time;
						leadspeed = vf; //initial speed
						speed = vf; // designed speed
					}
					else if(vnew > vmin)
					{
						double penalty = Math.max(lenIS/vf, (-vnew+ Math.sqrt(Math.pow(vnew,2.0) + 2*a*lenIS))/a);
						if (flag)
							deptime = Math.max(lastdeptime+1/cap+lenIS/vf, sig.startTime+lenIS/vf);
						else
							deptime = Math.max(-currentCar.location/vf+lenIS/vf+time, Math.max(lastdeptime+1/cap+penalty, sig.startTime+penalty));
						leadspeed = vnew;
						speed = vnew;
					}
					else
					{
						double penalty = Math.max(lenIS/vf, (Math.sqrt(Math.pow(vnew,2.0) + 2*a*lenIS))/a);
						if (flag)
							deptime = Math.max(lastdeptime+1/cap+lenIS/vf, sig.startTime+lenIS/vf);
						else
							deptime = Math.max(-currentCar.location/vf+lenIS/vf+time, Math.max(lastdeptime+1/cap+penalty, sig.startTime+penalty));		
						leadspeed = 0;
						speed = vmin;
					}

				}
				else
				{
					if(leadspeed0>vf-1e-3)
					{
						if (flag)
							deptime = Math.max(lastdeptime+1/cap+lenIS/vf, sig.startTime+lenIS/vf);
						else
							deptime = Math.max(-currentCar.location/vf+time+lenIS/vf, Math.max(lastdeptime+1/cap+lenIS/vf, sig.startTime+lenIS/vf));//byin
						speed = -1.0;
					}
					else
					{
						double vnew = Math.min(leadspeed0+(platoon-1)*a/cap, vf);
						double penalty = Math.max(lenIS/vf, (-vnew + Math.sqrt(Math.pow(vnew,2.0) + 2*a*lenIS))/a);
						if (flag)
							deptime = Math.max(lastdeptime+1/cap+lenIS/vf, sig.startTime+lenIS/vf);
						else
							deptime = Math.max(-currentCar.location/vf+lenIS/vf+time,Math.max(lastdeptime+1/cap+penalty, sig.startTime+penalty));
						leadspeed = 0;
						speed = -1.0;
					}

				}
			}
			else
			{
				double vnew = Math.min(leadspeed0+(platoon-1)*a/cap, vf);
				double penalty = Math.max(lenIS/vf, (-vnew + Math.sqrt(Math.pow(vnew,2.0) + 2*a*lenIS))/a);
				if (flag)
					deptime = Math.max(lastdeptime+1/cap+lenIS/vf, sig.startTime+lenIS/vf);
				else
					deptime = Math.max(lastdeptime+1/cap+penalty, sig.startTime+penalty);
				
				leadspeed = 0;
				speed = -1.0;
			}
		}
	}
	
	
	
	 double speed2(double pos, double time, double lastdeptime, double currentsignalstart)
	{
		double vnew =  pos/(Math.max(lastdeptime-time+1/cap+0.2, currentsignalstart-time+0.2));
		return vnew;
	}
	 
	 /*	double acceleration(double rl, double rs, double fl, double fs, double length) {
		double v = rs;
		double dv = v-fs; 																// speed difference (approaching grate)
		double s = fl - rl - length;
		double sdyn = minSpacing + v*TReact + Math.max(v*dv/(2*Math.sqrt(amax*ddes)),0);		// desired dynamic distance
		return amax*(1-Math.pow((1-fs/rs),delta)-Math.pow((sdyn/s),2));
	}*/
}
