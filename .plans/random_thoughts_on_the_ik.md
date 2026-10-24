
# Inverse Kinematics
So a platform mounted on a tensegrity like structure attached to three planets orbiting an 
internal ring gear.... not the usual IK chain.

The extra wrinkle is that the pose is partly defined by where the ball joints jam (that sounds disgusting when 
you write it down, but when you play with the physical device it feels legit). And that Jam changes by 
whatever cheap push rod linkage you get from ali-express or the local hobby-shop.

So finding a way to compensate for that variable is important. It seems reasonable that the jam angle 
will effect the platform height, so perhaps a simple measurement will do for calibration.

Also The platform can reach the sam pan/tilt positions in different ways, some of these poses the head 
is centered on the platform, and others it is off center. I don't really care for the off center poses, 
so they should be avoided.


# Considered Approaches

## 1) Typical model.

Geometrically work out how it all behaves, and build a model. Fun for a chain of joints, feels 
like a headache here.

## 2) Create lookup tables

Sweep the joint positions, and record head position and off center amount. Then invert and interpolate.
Probably adjust the jam angle and see how it alters the table. Perhaps the jams even out and its 
not a big deal, IDK.

Probably sweep tha jam angle and get a calibration graph for angle vs. platform height.

## 4) Train an AI to do it.

This is not a deep problem, like the system is not dynamically compensating for an environment.
It would probably just boil down tio a simple AI model that was less reliable than a look up table.

I do worry that inverted tables could result in a glitchy movement though. Some poses are reachable in 
multiple ways and smoothest path should win, and AI system might handle that better.

# IK problem 

The top platform will prob always be a bit of center, we have to find the "most central" solution 
to a given roll/pitch, that also tends to produces a nice (non-jumpy) result if given an interpolated 
path.

   - "most central" and "non-jumpy" might conflict, we want a stickiness so they don't jump.
   - The (lookaround) system feeds pose requests, not a path.So the desired solve may also have a 
     hysteresis where we look at the curve so far and plot a non-jumpy solution on where it appears
     to be heading.
   - is the jumpiness effect related to the length of the sled move? Mostly yeas, but playing with the 
     device in my hands if find near certain points the head can swing as the maximum torsion 
     changes rapid. I'm manipulating not on the sled harness, so IDK if that issue translates to
     the sled or not. 


# Other notes.

The robbie_scad sleds are bigger than the sleds I originally designed, to alow for the rotary 
encoder. Atheistically its not as nice, but also that small change ramped up the exclusion zone to
like 56deg.

