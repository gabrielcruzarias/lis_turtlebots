import rospy
from SimpleServer import SimpleServer, SimpleClient
from argparse import ArgumentParser
import time

class interface:
  START, PICKING, WAITING_TO_PLACE, PLACING= range(4)
  def __init__(self, init_state=None, debug=False):
      self.server  = SimpleServer(12345)
      self.client = SimpleClient(host="localhost",port=12346) #10.68.0.171
      
      if init_state == None:
          self.state = self.START
      else: 
          self.state = init_state

      self.event_loop()
      
  def event_loop(self):
      while True:
          rospy.loginfo("starting event loop!  current state = %d" % self.state)

          if self.state == self.START:
              self.state = self.PICKING
              self.pick_up()
              self.state = self.WAITING_TO_PLACE
          
          if self.state == self.WAITING_TO_PLACE:
              self.send_msg_to_turtle("pr2 ready to place can")
              self.wait_until_msg_is("turtle in place position")
              self.state = self.PLACING
          
          if self.state == self.PLACING:
              self.place()
              self.send_msg_to_turtle("pr2 placed object")
              #self.wait_until_msg_is("turtle left pr2")
              self.state = self.PICKING

          if self.state == self.PICKING:
              rospy.loginfo("rotating")
              self.pick_up()
              self.state = self.WAITING_TO_PLACE
              time.sleep(60)

  def wait_until_msg_is(self,correct_msg):
      rospy.loginfo("waiting to receive following msg from turtle: %s"\
      % correct_msg)
      msg = self.receive_msg_from_turtle()
      while msg != correct_msg:
          msg = self.receive_msg_from_turtle()
              
  def send_msg_to_turtle(self, msg):
      rospy.loginfo( "sending message: %s " % msg)
      self.server.broadcast(msg)
      rospy.loginfo( "message sent: %s " % msg)
  
  def receive_msg_from_turtle(self):
      msg_received = False
      while not msg_received:
          try:
              msg = self.client.get_message()
              msg_received = True
              rospy.loginfo("message received is: %s " % msg)
          except:
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


