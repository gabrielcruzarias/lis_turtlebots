

from threading import Thread
from SimpleServer import *
import time

class WaypointsServer(object):
    client_ports = {"donatello" : 12348, "leonardo" : 12349}
    client_hosts = {"donatello" : "10.68.0.171", "leonardo" : "10.68.0.171"}
    #client_hosts = {"donatello" : "localhost", "leonardo" : "localhost"} # no robot testing
    #client_hosts = {"donatello" : "10.68.0.165", "leonardo" : "10.68.0.165"} # no robot testing
    server_ports = {"donatello" : 12350, "leonardo" : 12351}
    def __init__(self):
        
        self.stop = False
        
        self.reserved_waypoints = {}
        
        self.client_requesters = {}
        self.client_responders = {}
        
        for client_name in self.client_ports.keys():
            self.client_requesters[client_name] = SimpleClient(host = self.client_hosts[client_name], port = self.client_ports[client_name])
            self.client_responders[client_name] = SimpleServer(port = self.server_ports[client_name], threading = False)
            t = Thread(target = self.loop, args = [client_name])
            t.start()
        
        while (not self.stop):
            raw_input("Hit enter to stop")
            self.stop = True
    
    def loop(self, client_name):
        while (not self.stop):
            try:
                (command, x, y) = self.client_requesters[client_name].get_message().split(",")
                t = time.time()
                if (command == "reserve"):
                    #print client_name + " is reserving waypoint Point" + str((x, y))
                    #dummy = True
                    #while (self.reserved_waypoints.has_key((x, y)) and self.reserved_waypooints[(x,y)] != client_name):
                        #if (dummy):
                        #    print self.reserved_waypoints[(x, y)] + " owns Point" + str((x, y)) + " at the moment. Waiting..."
                        #    dummy = False
                    #    time.sleep(0.3)
                    if (self.reserved_waypoints.has_key((x, y))):
                        print client_name + " is requesting Point(" + str((x, y))
                        self.reserved_waypoints[(x, y)] += [client_name]
                    else:
                        self.reserved_waypoints[(x, y)] = [client_name]
                        self.client_responders[client_name].broadcast("granted," + str(x) + "," + str(y))
                        print "Reserved waypoint Point" + str((x, y)) + " for " + client_name + ", waited for " + str(time.time() - t)
                else:
                    print client_name + " released waypoint Point" + str((x, y))
                    if (len(self.reserved_waypoints[(x, y)]) > 1):
                        self.reserved_waypoints[(x, y)].pop(0)
                        new_owner = self.reserved_waypoints[(x, y)][0]
                        print "New owner = ", new_owner
                        self.client_responders[new_owner].broadcast("granted," + str(x) + "," + str(y))
                    else:
                        print "Point(" + str((x, y)) + " has no owner anymore"
                        self.reserved_waypoints.pop((x, y))
            except:
                time.sleep(0.5)
        

if __name__=="__main__":
    waypoints_server = WaypointsServer()
