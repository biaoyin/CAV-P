

// class signalLight: behaviors of signals
	public class signalLight{
		double signalStart = Parameter.signalStart;
		
		int phase; // approach receiving the green light;
		double startTime; // the start time of the current green signal;
		signalLight() //constructor default
		{
			phase = 1;
			startTime = signalStart;
		}
		
		signalLight(int app, double stime) //constructor: given parameters
		{
			phase = app;
			startTime = stime;
		}
		
		void switchSignal(double time) //switch signal
		{
			startTime = time;
			phase =3- phase;
		}
		
		signalLight copy() // obtain a deep copy of the current signal
		{
			signalLight sig = new signalLight();
			sig.phase = phase;
			sig.startTime = startTime;
			
			return sig;
		}
		
		public String toString()
		{
			return ((Integer) phase).toString() + " " + ((Double) startTime).toString();
		}
	}
	