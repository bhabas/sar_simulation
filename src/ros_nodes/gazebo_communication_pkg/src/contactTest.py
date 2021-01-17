from logging import fatal
import rospy
from gazebo_msgs.msg import ContactsState

class Contact():
    def __init__(self):
        rospy.init_node("contact_node") 
        rospy.Subscriber('/ceiling_contact',ContactsState,self.contactCallback)
        self.modelName =  'crazyflie_model_X'



        self.pad_contact = [False,False,False,False]



    def contactCallback(self,msg_arr):
        for msg in msg_arr.states:
            if msg.collision1_name  ==  f"{self.modelName}::pad_1::collision" and self.pad_contact[0] == False:
                self.pad_contact[0] = True

            elif msg.collision1_name == f"{self.modelName}::pad_2::collision" and self.pad_contact[1] == False:
                self.pad_contact[1] = True

            elif msg.collision1_name == f"{self.modelName}::pad_3::collision" and self.pad_contact[2] == False:
                self.pad_contact[2] = True

            elif msg.collision1_name == f"{self.modelName}::pad_4::collision" and self.pad_contact[3] == False:
                self.pad_contact[3] = True

        
        # print(msg_arr.states)


if __name__ == '__main__':
    ct = Contact()

    while True:
        input()
        print(ct.pad_contact)
        ct.pad_contact = [False,False,False,False]

