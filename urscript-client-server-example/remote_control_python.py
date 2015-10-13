# Echo client program
import socket
import time
HOST = "192.168.1.213"      # The remote host
PORT = 30003                # The same port as used by the server

SEND_TIME = 1.0/125.0

print "Starting Program"
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)         # Create a socket object
count = 0
s.connect((HOST, PORT))        # Bind to the port
#s.listen(5)                 # Now wait for client connection.
#c, addr = s.accept()        # Establish connection with client.


time.sleep(0.05)
s.send("set_digital_out(2,True)" + "\n")
time.sleep(0.05)
print "0.2 seconds are up already"
s.send("set_digital_out(7,True)" + "\n")
   
while (count < 1):

    #print "Enter Command:"
    #str_to_send = raw_input("Enter Command:")
    #s.send(str_to_send + "\n")

    #s.send("movej([-0.5405182705025187, -2.350330184112267, -1.316631037266588, -2.2775736604458237, 3.3528323423665642, -1.2291967454894914], a=1.3962634015954636, v=1.0471975511965976)" + "\n")
    #time.sleep(6)

    # set_pos IS NOT VALID
    #s.send("movej([0, -1.57, 0, -1.57, 0, 0], a=1.395, v=1.047)" + "\n")
    #time.sleep(3)

    s.send("force_mode(p[0.0, -191.45, 1000.0, 0.0, 2.2215, -2.2215], [1, 1, 1, 1, 1, 1], [1.0, 1.0, 1.0, 0.0, 0.0, 0.0], 1, [1.2, 1.2, 1.2, 1.2, 1.2, 1.2])"+"\n")
    time.sleep(3)

    count = count + 1
    print "The count is:", count
    time.sleep(1)
    data = s.recv(1024)
    print data

s.send("set_digital_out(2,False)" + "\n")
time.sleep(0.05)
print "0.2 seconds are up already"
s.send("set_digital_out(7,False)" + "\n")
time.sleep(1)

s.close()


#c.close()

print "Program finish"
