import numpy
import struct
import time
import math

servoOutStruct = struct.Struct('<'+2*6*'d')
servoInStruct = struct.Struct('<'+3*6*'d')
servoOutPort = 2010                
servoInPort = 2011
import socket              
servoOut=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
servoOut.bind(('',2010))

#servoOut.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
servoOut.settimeout(1)

class FGen:
    def __init__(self, q0):
        self.t0 = time.time()
        self.q0=q0
        self.omega = numpy.array([0.,0.,0.,0.,0.,0.])
        self.omega[5] = math.pi
        self.omega[2] = math.pi/1.5
        self.omega[0] = math.pi/2
        self.ampl = numpy.array([0.,0.,0.,0.,0.,0.])
        self.ampl[5] = math.pi/8.
        self.ampl[2] = math.pi/7.
        self.ampl[0] = math.pi/5.
    def __call__(self):
        t=time.time()-self.t0
        sq = self.q0 - self.ampl + self.ampl*numpy.cos(t*self.omega)
        sdq = -(self.ampl*self.omega)*numpy.sin(t*self.omega)
        sddq = -(self.ampl*self.omega**2)*numpy.cos(t*self.omega)
        return numpy.array([sq,sdq,sddq]).flatten()
    
q=numpy.zeros(6)
dq=numpy.zeros(6)
ddq=numpy.zeros(6)
ip = None

while True:                                                                                                                
    try:
        pkg=servoOut.recvfrom(1024)
        aqdq=numpy.array(servoOutStruct.unpack(pkg[0]))
        aq=aqdq[:6]
        adq=aqdq[6:]
        # print 'Pos : '+6*'%2.3f ' % q
        # print 'Vel : '+6*'%2.3f ' % dq
        # print 
    except socket.timeout:
        print 'No connection!'
        ip = None
    else:
        if ip is None:
            print 'Got connection from %s' % str(pkg[1])
            q0 = aq
            fgen = FGen(q0)
            print 'Generating open loop wave with:'
            print '\tAmplitude: '+6*'%2.3f ' % tuple(fgen.ampl)
            print '\tOmega:      '+6*'%2.3f ' % tuple(fgen.omega)
            ip = pkg[1][0]
        servoOut.sendto(servoInStruct.pack(*fgen()),(ip,servoInPort))
