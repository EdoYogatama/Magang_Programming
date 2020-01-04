import numpy as np

def setup():
	global waypoints
	waypoints = np.loadtxt("bayucaraka.waypoints", dtype=float, skiprows=1, usecols=(8,9))
def gets():
	return waypoints
	
def get( index ):
        return waypoints[ index ]
