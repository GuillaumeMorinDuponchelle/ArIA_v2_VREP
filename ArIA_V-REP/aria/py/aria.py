import threading
import socket
import sys
import struct
import time
import signal
import numpy as np

"""
ArIA control 
Actuators : 18 servos
"""

class Aria:
    # interrupt handler
    def interrupt_handler(self,signal, frame):
        print ('You pressed Ctrl+C! Aria will stop in the next 3 seconds ')
        if self.vrep.isAlive():
            pass # clean stop should be placed here
            self.full_end()
        sys.exit(0)

    def __init__(self):
        # Connect the socket to the port where the server is listening on
        self.server_address = ('localhost', 30100)

        self.nJoint = 18
        self.val = np.zeros(self.nJoint)
        self.cmd = np.zeros(self.nJoint)
        self.pose = np.zeros(4) # x,y,z and heading

        #self.debug = True # display debug messages on console
        self.debug = False # do not display debug 

        self.dtmx = -1.0
        self.dtmn = 1e38
        self.cnt_sock = 0
        self.dta_sock = 0.0
        self.upd_sock = False
        self.aria_ready = threading.Event()
        self.aria_ready.clear()
 
        # initiate communication thread with V-Rep
        self.simulation_alive = True
        srv = self.server_address
        ev = self.aria_ready
        self.vrep = threading.Thread(target=self.vrep_com_socket,args=(srv,ev,))
        self.vrep.start()
        # wait for robot to be ready
        self.aria_ready.wait()
        print ("Aria ready ...")
        # trap hit of ctrl-x to stop robot and exit (emergency stop!)
        signal.signal(signal.SIGINT, self.interrupt_handler)

    def vrep_com_socket(aria,srv,ev):
        while True:
            #print ("update vrep with sock")
            #print (aria.simulation_alive,aria.speedLeft,aria.speedRight)
            # Create a TCP/IP socket
            t0 = time.time()
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            try:
                sock.connect(srv)
            except:
                print ("Simulation must be alive to execute your python program properly.")
                print ("Type Ctrl-C to exit, start the simulation and then execute your python program.")
                break

            sock.settimeout(0.5)
            vtx = aria.cmd
            nch = 4*aria.nJoint
            data = [ord(chr(59)),ord(chr(57)),nch%256,nch/256,0,0]
            strSend = struct.pack('<BBBBBB%df'%(aria.nJoint),data[0],data[1],data[2],data[3],data[4],data[5],*aria.cmd) 
            sock.sendall(strSend)
            upd_sock = True

            data = b''
            nrx = 6+4*4+aria.nJoint*4
            try:
                while len(data) < nrx:
                    data += sock.recv(nrx)
            except:
                print ("socker error , duration is %f ms, try to reconnect !!!"%((time.time() - t0)*1000.0))
                #sock.detach()
                #sock.connect(srv)
                #print ("socker error , type Ctrl-C to exit !!!")
                #exit(0)
            #print len(data),nrx
            if len(data) == nrx:
                vrx = struct.unpack('<ccHH%df'%(4+aria.nJoint),data)
                aria.vrep_update_sim_param(upd_sock,vrx)
            else:
                print ("bad data length ",len(data))


            sock.close()
            aria.cnt_sock = aria.cnt_sock + 1
            tsock = (time.time() - t0)*1000.0
            aria.dta_sock += tsock
            if tsock > aria.dtmx:
                aria.dtmx = tsock
            if tsock < aria.dtmn:
                aria.dtmn = tsock
            dtm = aria.dta_sock/float(aria.cnt_sock)
            #print ("tsock",tsock)
            if aria.debug:
                if (aria.cnt_sock % 100) == 99:
                    print ("min,mean,max socket thread duration (ms) : ",aria.dtmn,dtm,aria.dtmx)

            #time.sleep(0.2)
            #print (dir(ev))
            ev.set()

            if not aria.simulation_alive:
                break 

    def vrep_update_sim_param(self,upd_sock,vrx):
        #print (upd_sock)
        self.upd_sock = upd_sock
        i = 4
        for ip in range(4):
            self.pose[ip] = vrx[i]
            i += 1
        for ip in range(self.nJoint):
            self.val[ip] = vrx[i]
            i += 1
       
    def stop (self):
        """
        Stop the robot ... not implemented yet
        """
        pass

    def full_end (self):
        """
        Fully stop the simulation of the robot 
        stop the robot (not implemented yet)
        and close the connection with the simulator vrep
        sleep for 2 seconds to end cleanly 
        """
        pass
        time.sleep(2.0)
        print ("clean stop of  Aria")
        self.simulation_alive = False

    def set_single_joint (self,num,val):
        """
        num : joint number
        val : angle in degrees
        """
        self.cmd[num-1] = float(val)
        self.upd_sock = False
        #print (self.upd_sock)
        while not self.upd_sock:
            time.sleep(0.0001)
        #print (self.upd_sock)

    def get_single_joint (self,num):
        """
        return the values of a joint
        """
        return self.val[num-1]

    def set_multiple_joints (self,nums,vals):
        """
        nums : array with joint numbers
        vals : array with corresponding angles in degrees
        """
        for i in range(len(nums)):
            self.cmd[nums[i]-1] = float(vals[i])
        self.upd_sock = False
        #print (self.upd_sock)
        while not self.upd_sock:
            time.sleep(0.0001)
        #print (self.upd_sock)

    def get_multiplejoint (self,nums):
        """
        return the values of a list of joints defined in nums
        """
        vals = []
        for i in range(len(nums)):
            vals.append(self.val[nums[i]-1])
        return vals

       
if __name__ == "__main__":

    rb = Aria()
    print  rb.get_single_joint(1)
    rb.set_single_joint(1,20)
    time.sleep(2)
    print  rb.get_single_joint(1)    
    rb.set_single_joint(1,0)
    time.sleep(2)
    print  rb.get_single_joint(1)

    rb.set_multiple_joints([3,15,9,4,16,10],[50,-50,-50,-50,-50,50])
    print rb.get_multiple_joints([3,15,9,4,16,10])
    time.sleep(2)
    rb.set_multiple_joints([3,15,9,4,16,10],[0,0,0,0,0,0])
    print rb.get_multiple_joints([3,15,9,4,16,10])
     
    rb.full_end()


