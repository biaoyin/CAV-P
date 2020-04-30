import java.awt.Color;
import java.util.Random;


public class Car {
	
	// Parameters   
	double amax = Parameter.amax; //maximum acceleration 0.73
	double ddes = Parameter.ddes; // desired deceleration
	double delta = Parameter.delta; // acceleration exponent
	double minSpacing = Parameter.minSpacing; //minimum spacing
	double s0 = minSpacing; // jam distance;
	double vdes = Parameter.vdes; // desired speed m/s
	double TReact = Parameter.TReact; // driver reaction time
	double a = amax; // acceleration rate for crossing the intersection
	double vf = Parameter.vf; // speed
	double length = Parameter.length; // car length
	double cap = Parameter.cap; // saturation flow veh/s
	double zoneArrive = Parameter.zoneArrive; //  length of approach: from just arriving vehile to stopline
	double gmax = Parameter.gmax; // maximum green duration(s)
	double gmin = Parameter.gmin; // minimum green duration(s)
	double followGap = Parameter.followGap; // m
	double speedVar = Parameter.speedVar;
	double vmin = Parameter.vmin;	// minimum of cruise vehicle speed
	double lenIS = Parameter.lenIS;  // length of intersection
	double zoneLength = Parameter.zoneLength; // communication range
	int c = Parameter.c;	// Nb of vehicles generated
	int timeStep= Parameter.timeStep;	// simulation 0.025s per step default 25; if make it bigger (100), then faster yet little difference appears
	private Random random = new Random();


	// class of each car
	int id;
	int numstop;
	double speed; // car speed at this time step (m/s);
	double location; // relative location of the car front at this time step  (m)<0;
	int approach; // which lane the car is on
	boolean equipped; // whether it is equipped;
	boolean automated; // whether it is automated;
	double prevLocation; // relative location of the car front at the previous time step (m);
	double prevSpeed; // car speed at the previous time step (m/s);
	int idxList; //car idx in ArrivalList
	double acceleration;  // acceleration rate at this time step (m/s2);
	Color color; // color of the car;
	double virtualArrivalTime; // i.e., virtual departure time= arrive at downstream
	double enterTime;
	double noise;	
	int h = Parameter.h; // length of car in y coordinate (pixel);
	int w = Parameter.w; // length of car in x coordinate (pixel);	
		
		//boolean mode; // if mode is true, the car is following the instruction, otherwise the car is following the previous car.
		
		Car()
		{
			
		}
		
		
		Car(int ID, double v, double l,  double len, int app, boolean equip, boolean auto, int idx, double time) // constructor
		{
			numstop = 0;
			id = ID;
			speed = v;
			prevSpeed = v;
			location = l;
			prevLocation = l;
			length = len; //
			approach = app;
			equipped = equip;
			automated = auto;
			idxList = idx;
			virtualArrivalTime = zoneArrive/vf+lenIS/vf+time;
			enterTime = time;
			if( equipped && automated)
				color = Color.blue;
			else if(equipped)
				color = Color.cyan;
			else
				color = Color.yellow;			
				
		}
		
		
		void followFirst() // the first car in the zone, with green signal, without instruction
		{
			
			acceleration = a;
			prevSpeed = speed;
			speed = Math.min(speed + acceleration * (double)timeStep /1000.0, vf);
			prevLocation = location;
			double temptime = (speed-prevSpeed)/acceleration;
			location = location + prevSpeed * temptime + acceleration * temptime*temptime/2
					+speed*((double)timeStep/1000.0-temptime);
			if(speed<1e-3 && prevSpeed>=1e-3 && location >-zoneLength)
				numstop++;
		}
		void followNormal(Car c) // the following car in the zone, with green signal, without instruction
		{
			
			acceleration = IDM(location, speed, c.prevLocation, c.prevSpeed, c.length);
			prevSpeed = speed;
			speed = Math.min(speed + acceleration * (double)timeStep /1000.0, vf);
			double temptime = acceleration<1e-3?0:(speed-prevSpeed)/acceleration;
			if(speed < 0)
				speed = 0;
			prevLocation = location;
			location = location + prevSpeed * temptime + acceleration * temptime*temptime/2
					+speed*((double)timeStep/1000.0-temptime);
			if(c.location - location - c.length < minSpacing)
				location = c.location - c.length - minSpacing;
			if(speed<1e-3 && prevSpeed>=1e-3 && location >-zoneLength)
				numstop++;
		}	
		
		void followRedLight() // if the car is the leading car, and the signal light is red
		{
			Car c = new Car();
			c.location = length+minSpacing;
			c.speed = 0;
			c.prevLocation = length+minSpacing;
			c.prevSpeed = 0;
			if(c.location - location - c.length > followGap)
			{
				acceleration = 0;
				prevLocation = location;
				prevSpeed = speed;
				location = prevSpeed * (double)timeStep /1000.0 + location;
			}
			else
			{
				acceleration = IDM(location, speed, c.prevLocation, c.prevSpeed, c.length);
				prevSpeed = speed;
				speed = Math.min(speed + acceleration * (double)timeStep /1000.0, vf);
				if(speed < 0)
					speed = 0;
				double temptime = acceleration<1e-3?0:(speed-prevSpeed)/acceleration;
				
				prevLocation = location;
				location = location + prevSpeed * temptime + acceleration * temptime*temptime/2
						+speed*((double)timeStep/1000.0-temptime);
				if(location  >0)
				{
					speed = 0;
					location = 0;
				}
			}
			if(speed<1e-3 && prevSpeed>=1e-3 && location >-zoneLength)
				numstop++;
			
			//else if (c.location - location - c.length > minSpacing && speed<1e-3)
			//	speed = 1; 
		}
		
		void followInstruction(double v, Car c) // if the car is following the instruction
		{
			prevSpeed = speed;
			prevLocation = location;
			if(c==null) // the first car
			{
				speed = v;
				acceleration = 0;
				location = speed * (double)timeStep /1000.0 + location;
			}
			else
			{
				double acc = IDM(location, speed, c.prevLocation, c.prevSpeed, c.length);
				
				if(c.location - location - c.length > followGap) // the car is far away from the previous car
				{
					speed = v;
					acceleration = 0;
					location = speed * (double)timeStep /1000.0 + location;
				}
				else if(speed + acc*(double)timeStep /1000.0 > v) // the assigned speed is smaller than the following speed, i.e. the car won't hit the previous car
				{
					acceleration = 0;
					speed = v;
					location = speed * (double)timeStep /1000.0 + location;
				}
				else // the assigned speed is larger than the following speed and the car is close to the previous car, the car will follow the previous car
				{
					acceleration = acc;
					speed = Math.min(speed + acceleration * (double)timeStep /1000.0, vf);
					if(speed < 0)
						speed = 0;
					double temptime = acceleration<1e-3?0:(speed-prevSpeed)/acceleration;
					location = location + prevSpeed * temptime + acceleration * temptime*temptime/2
							+speed*((double)timeStep/1000.0-temptime);
				}
			}
			if(speed<1e-3 && prevSpeed>=1e-3 && location >-zoneLength)
				numstop++;
			
			
		}
		
		void followInstructionRedLight(double v) // if the car is following the instruction and the signal light is red
		{
			Car c = new Car();
			c.location = length+minSpacing;
			c.speed = 0;
			c.prevLocation = length+minSpacing;
			c.prevSpeed = 0;
			double acc = IDM(location, speed, c.prevLocation, c.prevSpeed, c.length);
			prevSpeed = speed;
			prevLocation = location;
			if(c.location - location - c.length > followGap) // the car is far away from the previous car
			{
				speed = v;
				acceleration = 0;
				location = prevSpeed * (double)timeStep /1000.0 + location;
			}
			else if(speed + acc*(double)timeStep /1000.0 > v) // the assigned speed is smaller than the following speed, i.e. the car won't hit the previous car
			{
				acceleration = 0;
				speed = v;
				location = prevSpeed * (double)timeStep /1000.0 + location;
				if(location  >0)
				{
					speed = 0;
					location = 0;
				}
			}
			else // the assigned speed is larger than the following speed and the car is close to the previous car, the car will follow the previous car
			{
				speed = Math.min(speed + acceleration * (double)timeStep /1000.0, vf);
				if(speed < 0)
					speed = 0;
				double temptime = acceleration<1e-3?0:(speed-prevSpeed)/acceleration;
				location = location + prevSpeed * temptime + acceleration * temptime*temptime/2
						+speed*((double)timeStep/1000.0-temptime);
				if(location  >0)
				{
					speed = 0;
					location = 0;
				}
			}
			
			if(speed<1e-3 && prevSpeed>=1e-3 && location >-zoneLength)
				numstop++;
			
		}
		
		boolean stop()
		{
			return speed < 0.1;
		}
		
		double IDM(double rl, double rs, double fl, double fs, double length) // car following model
		{
			/* The parameters: 
			 * rl - location of the rear car
			 * rs - speed of the rear car
			 * fl - location of the front car
			 * fs - speed of the front car
			 * length - length of the front car
			 * The front car follows the rear car
			 * The location of each car increases as it runs. 
			 */
			
			double ss = s0+rs*TReact+rs*(rs-fs)/2/Math.sqrt(amax*ddes);
			double acc = amax*(1-Math.pow(rs/vdes, delta)-Math.pow(ss/(fl-rl-length), 2));
			// TODO: update acc according to IDM, add any global variable you need to the top. 
			
			return acc + random.nextGaussian()*speedVar;
		}	
	}