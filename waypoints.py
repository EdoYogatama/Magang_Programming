import numpy as np
import re

# Const
WAYPOINTS_FILENAME				= 'vtol.waypoints'
GENERATED_WAYPOINTS_FILENAME	= 'vtol_gen.waypoints'

waypoints			= []
home				= []

def setup():
	global waypoints, home
	try:
		with open(GENERATED_WAYPOINTS_FILENAME, 'r') as f:
			f.readlines()

		waypoints = np.genfromtxt(GENERATED_WAYPOINTS_FILENAME, dtype=float, skip_header=1)
	except FileNotFoundError:
		waypoints = np.genfromtxt(WAYPOINTS_FILENAME, dtype=float, skip_header=1, usecols=(8, 9, 10))

	home = waypoints[0]
	waypoints = waypoints[1:]

	save()

	return waypoints
	
def gets():
	return waypoints
	
def get(index):
	return waypoints[ index ]

def get_home():
	return home

def save():
	with open(GENERATED_WAYPOINTS_FILENAME, 'w') as f:
		f.write('VTOL SOEROMIBER BAYUCARAKA ITS GENERATED WAYPOINT LIST\n')
		f.write('\t'.join([str(item) for item in home]))
		f.write('\n')
		for sublist in waypoints:
			f.write('\t'.join([str(item) for item in sublist]))
			f.write('\n')

def clearwp():
	global waypoints
	waypoints = []

def insert(lat, lon, alt):
	global waypoints
	np.append(waypoints, [lat, lon, alt])

def set_home(lat, lon, alt):
	global home
	home = [lat, lon, alt]

def print_all():
	global waypoints
	for i in range(len(waypoints)):
		print(waypoints[i])