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
   
while (count < 1):
	
    time.sleep(0.05)
    s.send("set_digital_out(2,True)" + "\n")
    time.sleep(0.05)
    print "0.2 seconds are up already"
    s.send("set_digital_out(7,True)" + "\n")
    time.sleep(2)

    s.send("movej([-0.5405182705025187, -2.350330184112267, -1.316631037266588, -2.2775736604458237, 3.3528323423665642, -1.2291967454894914], a=1.3962634015954636, v=1.0471975511965976)" + "\n")
    time.sleep(2)
    s.send("movej([-0.5405182705025187, -2.350330184112267, -1.316631037266588, -2.2775736604458237, 3.3528323423665642, -1.2291967454894914], a=1.3962634015954636, v=1.0471975511965976)" + "\n")
    time.sleep(2)


    s.send("set_digital_out(2,False)" + "\n")
    time.sleep(0.05)
    print "0.2 seconds are up already"
    s.send("set_digital_out(7,False)" + "\n")
    time.sleep(1)

    count = count + 1
    print "The count is:", count
    time.sleep(1)
    data = s.recv(1024)
    print("Recieved", repr(data))

s.close()


#c.close()

print "Program finish"
