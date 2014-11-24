import rospy
from SimpleServer import SimpleServer, SimpleClient
from argparse import ArgumentParser
import time
import random

class interface:
  START, PICKING, WAITING_TO_PLACE, PLACING= range(4)
  def __init__(self, init_state=None, debug=False):
      self.server  = SimpleServer(port=12345, threading=True)
      self.clientD = SimpleClient(host="localhost",port=12346) #10.68.0.171 # D for Donatello
      self.clientL = SimpleClient(host="localhost",port=12347) # L for Leonardo
      self.turtle_being_attended = ""
      
      if init_state == None:
          self.state = self.START
      else:
          self.state = init_state

      self.event_loop()
      
  def event_loop(self):
      while True:
          rospy.loginfo("starting event loop!  current state = %d" % self.state)

          if self.state == self.START:
              raw_input("Hit enter to start...")
              self.state = self.PICKING
              self.server.update_broadcast("picking")
              self.pick_up()
              self.state = self.WAITING_TO_PLACE
          
          if self.state == self.WAITING_TO_PLACE:
              self.send_msg_to_turtle("pr2 ready to place can")
              self.wait_until_msg_is("can i come")
              self.send_msg_to_turtle("come " + self.turtle_being_attended)
              rospy.loginfo("Waiting for " + self.turtle_being_attended)
              self.wait_until_msg_is("turtle in place position", self.turtle_being_attended)
              self.state = self.PLACING
          
          if self.state == self.PLACING:
              self.send_msg_to_turtle("placing")
              self.place()
              self.send_msg_to_turtle("pr2 placed object")
              self.wait_until_msg_is("turtle left pr2", self.turtle_being_attended)
              self.state = self.PICKING

          if self.state == self.PICKING:
              self.send_msg_to_turtle("picking")
              self.pick_up()
              self.state = self.WAITING_TO_PLACE
              
  def wait_until_msg_is(self,correct_msg, name="turtle"):
      rospy.loginfo("waiting to receive following msg from turtle: %s"\
      % correct_msg)
      msg = self.receive_msg_from(name)
      while msg != correct_msg:
          msg = self.receive_msg_from(name)
      print "msg = ", msg, ", turtle = ", self.turtle_being_attended
      
  def send_msg_to_turtle(self, msg):
      rospy.loginfo( "sending message: %s " % msg)
      self.server.update_broadcast(msg)
      rospy.loginfo( "message sent: %s " % msg)

  def receive_msg_from(self, name):
      msg_received = False
      msg = None
      while not msg_received:
          try:
              if (name == "donatello"):
                  msg = self.clientD.get_message()
                  self.turtle_being_attended = "donatello"
                  #rospy.loginfo("message received from donatello is: %s " % msg)
              elif (name == "leonardo"):
                  msg = self.clientL.get_message()
                  self.turtle_being_attended = "leonardo"
                  #rospy.loginfo("message received from leonardo is: %s " % msg)
              else:
                  msg = self.receive_msg_from_turtle()
              msg_received = True
          except:
              rospy.sleep(1)
      return msg

  def receive_msg_from_turtle(self):
      msg_received = False
      msg = None
      while not msg_received:
          try:
              msgD = self.clientD.get_message()
              #rospy.loginfo("message received from donatello is: %s " % msgD)
          except:
              pass
          try:
              msgL = self.clientL.get_message()
              #rospy.loginfo("message received from leonardo is: %s " % msgL)
          except:
              pass
          if (msgD != None and msgL != None):
              msg_received = True
              if (random.randint(0, 1) == 0):
                  msg = msgD
                  self.turtle_being_attended = "donatello"
              else:
                  msg = msgL
                  self.turtle_being_attended = "leonardo"
          elif (msgD != None):
              msg = msgD
              self.turtle_being_attended = "donatello"
              msg_received = True
          elif (msgL != None):
              msg = msgL
              self.turtle_being_attended = "leonardo"
              msg_received = True
          else:
              rospy.sleep(1)
      return msg

  def pick_up(self):
      rospy.loginfo("picking up object")
      rospy.sleep(1) # dummy action 

  def place(self):
      rospy.loginfo("placing can")
      rospy.sleep(1) # dummy action

def awesome_parse_arguments():
    parser = ArgumentParser("Select initial state")
    parser.add_argument("--wait", action="store_true",
        help="wait for msg then place")
    parser.add_argument("--place", action="store_true",
        help="in place position")

    args = parser.parse_args()
    init_state = 0
    if args.wait:
        init_state=  2
    if args.place:
        init_state = 3
    return init_state
if __name__=="__main__":
  rospy.init_node("pr2_communication")
  init_state = awesome_parse_arguments()
  interface(init_state)


