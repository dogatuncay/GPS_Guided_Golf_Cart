#!/usr/bin/env python

import logging
import optparse
import os
import traceback

from geopy import distance                                                      
from cartfs import CartFSLoggingHandler, Sensor
from math import sqrt, atan, radians, cos, sin

# Default values for options
class options:
    # Cartfs File System Options
    debug = False
    root = '/tmp/cartfs'
    clock = 'clock'
    command = './driver/driver_c'
    status = './driver/driver_s'
    doc = './driver/driver_d'
    log = './driver/driver_log'

    vcs = './vcs/vcs_s'
    compass = './compass/compass_s'
    gps = './gps/gps_s'
    jdriver = './jdriver/jdriver_s'
    synlaser = './synlaser/synlaser_s'

class SquareDriver(Sensor):

    driver_c = {
        'clock': 0,
        'enable': True,
    }

    driver_c_doc = {
        'clock': "The clock value on which this data was written.",
        'enable': "True/False - stops reads on jdriver device",
    }

    driver_s = {
        'clock': 0,
        'enable': True,
        'direction': 0,
        'side_length': 10, # meters
        'corner_radius': 3, # meters
        'turn_state': 'turn',
        'turn_slop': 25, # degrees
        'turn_P_term': 1 # 
    }

    driver_s_doc = {
        'clock': "The clock value on which this data was written.",
        'enable': "True/False - stops reads on driver device",
        'direction': "The target heading of the driver",
        'side_length': "How long a side of the square should be (meters)",
        'corner_radius': "How large of a radius the corners should be (meters)",
        'turn_state': "What state of the turn we are in (turn/straight)",
        'turn_slop': "How close to straight we must be to be going straight",
        'turn_P_term': "The P term of the PID controller for steering",
    }

    jdriver_s = {
        'clock': 0,
        'enable': True,
        'percent_throttle': 0.0,
        'percent_braking': 0.0,
        'turn_radius_inverse': 0.0,
        'direction': 'forward',
        'mode': 'manual'
    }

    def __init__(self, options):
        Sensor.__init__(self)

        self.add_reader(options.clock, 'clock')
        self.add_reader(options.vcs, 'vcs_s')
        self.add_reader(options.compass, 'compass_s')
        self.add_reader(options.gps, 'gps_s')

        self.add_reader(options.command, 'driver_c')
        self.add_writer(options.status, 'driver_s', create=True)
        self.add_writer(options.jdriver, 'jdriver_s', create=True)
        self.add_reader(options.synlaser, 'synlaser_s')

        self.write_once(options.command, self.driver_c, create=True)
        self.write_once(options.command + '_d', self.driver_c_doc,
                        create=True, write_always=True)
        self.write_once(options.status + '_d', self.driver_s_doc,
                        create=True, write_always=True)
	self.waypoint = 0
	self.syntpointnum = 0
	self.syntpointdone='F'
	self.obs_found = 'F'
        self.obs_count = 0 
	self.manual_obs_count = 0
	self.counter ='turn_m'
	self.rounds=0

	self.course = [[1,39.181917,-86.5221208333,1.5,3.0],\
		    [2,39.1818975,-86.521724,1.5,3.0],\
                    [3,39.182143,-86.5217033333,1.5,3.0],\
                    [4,39.182199,-86.5220985,1.5,3.0],\
                    [5,39.1819156667,-86.522309,1.5,3.0],\
                    [6,39.1819645,-86.522398,1.5,3.0],\
                    [7,39.1820415,-86.5223095,1.5,3.0],\
                    [8,39.1821313333,-86.5223926667,1.5,3.0],\
                    [9,39.1822116667,-86.522302,1.5,3.0]]
	   
	self.syntpoint = []
	self.waypoint_latlon = []
	self.synt_latlon=[]

    def calc_inv_turn(self, desired_heading):


        heading = self.compass_s['heading']
        
        # Heading between -180, 180
        if heading > 180:
            heading -= 360.0

	# Error between heading and the desired heading
        diff = float(heading) - float(desired_heading)

        # Heading error between -180 and 180
        if diff > 180.0:
            diff -= 360.0
        elif diff < -180.0:
            diff += 360.0

        # Calculate the p term 
        turn_m = self.driver_s['turn_P_term'] * diff

        # Convert heading error pre-calculate
        turn_m = turn_m / 180.0

        # Take the smallest turn 
        max_turn = 1 / float(self.driver_s['corner_radius'])

        # Turn to (max_turn, -max_turn) 
        if turn_m > max_turn:
            turn_m = max_turn
        elif turn_m < -max_turn:
            turn_m = -max_turn

        return turn_m, diff

    def get_next_turn_radius_inverse(self):

	if self.waypoint%len(self.course)==0:
		if self.counter =='turn_m':
			self.rounds=self.rounds+1
			self.counter='F'

        heading = self.compass_s['heading']
        distance1 = self.vcs_s['distance']
        turn_slop = self.driver_s['turn_slop']
        turn_state = self.driver_s['turn_state']

	current_coordinate = [self.gps_s['lat'],self.gps_s['lon']]
	current_coordinate_2=[self.course[(self.waypoint)%len(self.course)][1],self.course[(self.waypoint)%len(self.course)][2]]
	current_coordinate_22=[self.course[(self.waypoint+1)%len(self.course)][1],self.course[(self.waypoint+1)%len(self.course)][2]]

#in kilometers
	d=distance.distance(current_coordinate_2,current_coordinate_22).kilometers * 1000
	
	distance_left = distance.distance(current_coordinate_22,current_coordinate).kilometers*1000

#Speed Control
	if(self.rounds == 4):
		self.jdriver_s['percent_throttle'] = 0.0 
	        self.jdriver_s['percent_braking'] = 100.0
	elif distance_left > 30:
		self.jdriver_s['percent_throttle'] = 70.0 
	        self.jdriver_s['percent_braking'] = 0.0  
		self.driver_s['turn_P_term'] = 1.0
	elif distance_left > 20:
		self.jdriver_s['percent_throttle'] = 60.0 
                self.jdriver_s['percent_braking'] = 0.0   
		self.driver_s['turn_P_term'] = 1.0
	elif distance_left > 10:
		self.jdriver_s['percent_throttle'] = 60.0 
                self.jdriver_s['percent_braking'] = 30.0   
		self.driver_s['turn_P_term'] = 2.0
	else:
		self.jdriver_s['percent_throttle'] = 50.0 
                self.jdriver_s['percent_braking'] = 30.0 
 
		if self.waypoint%len(self.course)==7 :
			self.driver_s['turn_P_term'] = 5.0
		else:
			self.driver_s['turn_P_term'] = 2.0	
#get the obstacle list
	self.obstacle = self.synlaser_s['obstacle_list']
	if self.obstacle == []:
		x_axis_difference = 1
     	 	y_axis_difference = 1
	else:
		x_axis_difference = self.gps_s['lat'] - self.obstacle[self.obs_count][1][0]
		y_axis_difference = self.gps_s['lon'] - self.obstacle[self.obs_count][1][1]
	
	dist = sqrt(x_axis_difference*x_axis_difference + y_axis_difference*y_axis_difference)

	if (dist<=.0015 ):
		self.obs_found = 'turn_m'
	else:
		self.obs_found = 'F'
# Here we calculate the synthetic points' coordinates
	x1=(self.course[(self.waypoint+1)%len(self.course)][1]+7*(self.course[(self.waypoint)%len(self.course)][1]))/8
	y1=(self.course[(self.waypoint+1)%len(self.course)][2]+7*(self.course[(self.waypoint)%len(self.course)][2]))/8
	
	x2=(self.course[(self.waypoint+1)%len(self.course)][1]+3*(self.course[(self.waypoint)%len(self.course)][1]))/4
	y2=(self.course[(self.waypoint+1)%len(self.course)][2]+3*(self.course[(self.waypoint)%len(self.course)][2]))/4

	x3=(self.course[(self.waypoint)%len(self.course)][1]+(self.course[(self.waypoint+1)%len(self.course)][1]))/2
	y3=(self.course[(self.waypoint)%len(self.course)][2]+(self.course[(self.waypoint+1)%len(self.course)][2]))/2	


	wp_coordinates = (self.course[(self.waypoint)%len(self.course)][1], self.course[(self.waypoint)%len(self.course)][2])
	wpp_coordinates = (self.course[(self.waypoint + 1)%len(self.course)][1], self.course[(self.waypoint + 1)%len(self.course)][2])
	wpn_coordinates = (self.course[(self.waypoint + 2)%len(self.course)][1], self.course[(self.waypoint + 2)%len(self.course)][2])

	distance_wpp = distance.distance(wp_coordinates,wpp_coordinates).forward_azimuth
	distance_wpn = distance.distance(wpp_coordinates,wpn_coordinates).forward_azimuth

	compass_decimal1=distance_wpp
	compass_decimal2=distance_wpn

	difference_y_axis = self.course[(self.waypoint + 1)%len(self.course)][2] - self.course[(self.waypoint)%len(self.course)][2]
	difference_x_axis = self.course[(self.waypoint + 1)%len(self.course)][1] - self.course[(self.waypoint)%len(self.course)][1]

	arctan_angle = atan(difference_y_axis/difference_x_axis)
	if arctan_angle < 0:
		arctan_angle = arctan_angle + 360
	rad = radians(arctan_angle)
	x4 =(self.course[(self.waypoint)%len(self.course)][1]+7*(self.course[(self.waypoint+1)%len(self.course)][1]))/8
	y4=(self.course[(self.waypoint)%len(self.course)][2]+7*(self.course[(self.waypoint+1)%len(self.course)][2]))/8
	x_d = abs((cos(rad) * 1.5)/100000)
	y_d = abs(1.5/(100000 * cos(rad)))

	if (0<=compass_decimal1<90):	
		if(90<=compass_decimal2<180):						y4 = y4 - y_d
		elif(270<=compass_decimal2<360):
						y4 = y4 +y_d									
	elif(180<=compass_decimal1<270):


		if(90<=compass_decimal2<180):
						y4 = y4 -y_d					
						
		elif(270<=compass_decimal2<360):
						y4 = y4 + y_d					
									
	elif(90<=compass_decimal1<180):


		if (0<=compass_decimal2<90):
						x4 = x4 - x_d					
						
		elif(180<=compass_decimal2<270):
						x4 = x4 + x_d										
						
	elif(270<=compass_decimal1<360):


		if (0<=compass_decimal2<90):
						x4 = x4 - x_d										
						
		elif(180<=compass_decimal2<270):
						x4 = x4 + x_d										

#add synthetic points			
	self.syntpoint=[[x1,y1],[x2, y2],[x3,y3],[x4,y4]]

	if self.obs_found == 'F':
		if ( d>20 and self.syntpointdone=='F'):
			
			syntpoint_latlon=[self.syntpoint[self.syntpointnum][0],self.syntpoint[self.syntpointnum][1]]
			
			d_btw_curr_and_synt=distance.distance(current_coordinate,syntpoint_latlon).kilometers * 1000
			if(d_btw_curr_and_synt<=3):
				if(self.syntpointnum < 3):
					self.synt_latlon=[self.syntpoint[self.syntpointnum+1][0],self.syntpoint[self.syntpointnum+1][1]]
					self.waypoint_latlon=self.synt_latlon
					self.syntpointdone='F'
					self.syntpointnum=(self.syntpointnum+1)%4
				elif (self.syntpointnum==3):
					self.waypoint_latlon = [self.course[(self.waypoint+1)%len(self.course)][1],self.course[(self.waypoint+1)%len(self.course)][2]]
					self.syntpointnum=0
					self.syntpointdone='turn_m'
			else:
				self.synt_latlon=[self.syntpoint[self.syntpointnum][0],self.syntpoint[self.syntpointnum][1]]
				self.waypoint_latlon=self.synt_latlon
			
		
		else:			
			self.waypoint_latlon = [self.course[(self.waypoint+1)%len(self.course)][1],self.course[(self.waypoint+1)%len(self.course)][2]]
					
		
	else:
		x_axis_diff_nxt_wp = self.gps_s['lat'] - self.obstacle[self.obs_count][1][0]
		y_axis_diff_nxt_wp = self.gps_s['lon'] - self.obstacle[self.obs_count][1][1]
		line_equa=(self.waypoint_latlon[1]-self.gps_s['lon'])-((y_axis_diff_nxt_wp *self.waypoint_latlon[0]-self.gps_s['lat'])/ x_axis_diff_nxt_wp )
		
		if  self.obstacle[self.obs_count][1][0] >= self.course[(self.waypoint+1)%len(self.course)][1]:
			if self.obstacle[self.obs_count][1][1] <= self.course[(self.waypoint+1)%len(self.course)][2]:			
				self.synt_latlon = [self.obstacle[self.obs_count][1][0] + .000013, self.obstacle[self.obs_count][1][1] + .000013]
			else:
				self.synt_latlon = [self.obstacle[self.obs_count][1][0] + .000013, self.obstacle[self.obs_count][1][1] - .000013]
		else:
			if self.obstacle[self.obs_count][1][1] <= self.course[(self.waypoint+1)%len(self.course)][2]:			
				self.synt_latlon = [self.obstacle[self.obs_count][1][0] - .000013, self.obstacle[self.obs_count][1][1] + .000013]
			else:
				self.synt_latlon = [self.obstacle[self.obs_count][1][0] - .000013, self.obstacle[self.obs_count][1][1] - .000013]
		
		self.waypoint_latlon = self.synt_latlon
		temp = [self.syntpoint[self.syntpointnum][0],self.syntpoint[self.syntpointnum][1]]
		d_btw_curr_and_synt=distance.distance(current_coordinate,temp).kilometers * 1000

		if(d_btw_curr_and_synt<2):
			self.syntpointnum=self.syntpointnum+1

		if self.syntpointnum == 4:
			self.syntpointnum == 3

	distance_left_tt_ms = distance.distance(current_coordinate,self.waypoint_latlon).kilometers * 1000
    	heading_left_tt_ms = distance.distance(current_coordinate,self.waypoint_latlon).forward_azimuth


        # Calcuate the next steering input
        turn, diff = self.calc_inv_turn(heading_left_tt_ms)

        # Check which part of the square we are on
        if turn_state == 'turn':
            # Check whether we have finished the corner
            if abs(diff) <= turn_slop:
                #print 'straight'
                self.driver_s['turn_state'] = 'straight'
                self.driver_s['straight_started_at'] = distance1
            pass
        elif turn_state == 'straight':
		if distance_left_tt_ms <= 3:
			self.mode='turn'
			if (self.waypoint_latlon==self.synt_latlon):
				#print 'waypoint_latlon = self.synt_latlon'
				if self.obs_found == 'turn_m':
					self.obs_found == 'F'
					self.obs_count == (self.obs_count + 1)% len(self.obstacle)
				else:
					if(self.syntpointnum==3 ):
						self.syntpointdone='turn_m'
						self.syntpointnum=0
						self.waypoint = (self.waypoint + 1)%len(self.course)
						
					else:
						self.syntpointdone='F'
			else:
				self.syntpointdone='F'
				self.waypoint = (self.waypoint + 1)%len(self.course)
				self.counter='turn_m'
				self.syntpointnum=0
				
        return turn

    def process(self):
        # Ensure there is a clock value in driver_s	
        if 'clock' not in self.driver_s:
            self.driver_s['clock'] = self.clock['clock'] - 1

        # Verify how many clock ticks have passed
        ticks = self.clock['clock'] - self.driver_s['clock']
        if ticks > 1:
            logging.error('missed %d clock cycles' % (ticks - 1,))


        # Test whether there are exceptions
        try:
            # Adjust the status values of the 'jdriver' sensor, which we are
            # simulating to make use of the simpler control values
            self.jdriver_s['mode'] = 'auto'           # autonomous mode
            self.jdriver_s['percent_throttle'] = 50.0 # set to keep speed relatively low
            self.jdriver_s['percent_braking'] = 0.0   # brakes off

            # Pass along whether we are enabled
            self.jdriver_s['enable'] = self.driver_c['enable']

            # Get next steering control
            self.jdriver_s['turn_radius_inverse'] = self.get_next_turn_radius_inverse()

 
	    print self.compass_s['heading'], \
                  self.vcs_s['distance'], self.vcs_s['speed'],\
                  self.gps_s['lat'], self.gps_s['lon']
        except Exception:
            # Print the traceback so we can still debug
            traceback.print_exc()

if __name__ == '__main__':
    # Use psyco to speed up
    try:
        import psyco
        psyco.full()
    except ImportError:
        pass



    # The command-line options for this module
    parser = optparse.OptionParser()
    parser.add_option('-d', '--debug', action='store_true', dest='debug')
    parser.add_option('-r', '--root', dest='root', metavar='ROOT')
    parser.add_option('-c', '--clock', dest='clock', metavar='CLOCK')
    parser.add_option('-C', '--command', dest='command', metavar='COMMAND')
    parser.add_option('-s', '--status', dest='status', metavar='STATUS')
    parser.add_option('-l', '--log', dest='log', metavar='LOG')
    parser.set_defaults(**options.__dict__)
    (options, args) = parser.parse_args()

    # Change directory into the root of the filesystem
    os.chdir(options.root)

    # Setup the debugging message format
    formatter = logging.Formatter('%(asctime)s %(name)-12s %(levelname)-8s %(message)s')

    # Setup the logging handler for logging to the filesystem
    log = CartFSLoggingHandler(options.clock, options.log)
    log.setLevel(logging.ERROR)
    log.setFormatter(formatter)
    logging.getLogger('').addHandler(log)

    # If we're in debug mode, then log to the console too
    if options.debug:
        console = logging.StreamHandler()
        console.setLevel(logging.DEBUG)
        console.setFormatter(formatter)
        logging.getLogger('').addHandler(console)

    # Instaniate our driver and run it
    sensor = SquareDriver(options)
    sensor.run()
