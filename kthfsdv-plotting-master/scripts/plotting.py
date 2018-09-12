#!/usr/bin/env python
import numpy as np
import math
import matplotlib.pyplot as plt
import time
import random
import json
import datetime
from tornado import websocket, web, ioloop
from datetime import timedelta
from random import randint
import os
import logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)



class Plotting():

    """"def __init__(self):
                        #self.n = n
                        #self.h = np.zeros(self.n)
                        #self.t = np.arange(self.h.size) # develop vector of 0,1,2,3,4,...,n-1
                        #self.counter = 0"""

    def callback(self, data):
        rospy.loginfo("{}".format(data.data))
        new_result = data.data / self.Q
        self.publish_result.publish(new_result)

        
    #def create_function(self, time):
         

class WebSocketHandler(websocket.WebSocketHandler):
  #on open of this socket
  def check_origin(self, origin):
    return True

  def open(self):
    print 'Connection established.'
    #ioloop to wait for 3 seconds before starting to send data
    ioloop.IOLoop.instance().add_timeout(datetime.       
    timedelta(seconds=20), self.send_data)
 
 #close connection
  def on_close(self):
    print 'Connection closed.'

  # Our function to send new (random) data for charts
  def send_data(self):
    print "Sending Data"
    #plotting = Plotting()
    while True:
        t0 = time.clock()
        ambda = 5 * np.sin(2*math.pi*t0) 
        h = 3 * math.pi * np.exp(-1*ambda)
        #value = plotting.create_function(t0)    
        #create a new data point
        point_data = {
            'sample_time': t0,
            'amplitude' : h
        }
        logger.info('{}'.format(point_data))
  
    #write the json object to the socket
    self.write_message(json.dumps(point_data))
    
    #create new ioloop instance to intermittently publish data
    ioloop.IOLoop.instance().add_timeout(datetime.timedelta(seconds=1), self.send_data)

if __name__ == '__main__':
    #plotting = Plotting()
    #plotting.create_function()

    print "Starting websocket server program. Awaiting client requests to open websocket ..."
    #application = web.Application([(r'/websocket', WebSocketHandler)])
    application = web.Application([(r'/static/(.*)', web.StaticFileHandler, {'path': os.path.dirname(__file__)}),
                                 (r'/websocket', WebSocketHandler)])
    application.listen(8888)
    ioloop.IOLoop.instance().start()