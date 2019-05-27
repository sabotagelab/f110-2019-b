#!/usr/bin/env python

import rospy
import csv
from std_msgs.msg import String
from speed_daemons_wall_following.msg import turn_instruction


class ReadInstruction:
    def __init__(self):
        rospy.init_node('instruction_reader_node', anonymous=True)
        rospy.Subscriber("instruction_feedback", String, self.instruction_recieved)
        self.pub = rospy.Publisher('drive_instruction', turn_instruction, queue_size=10)
        self.instructions = []
        self.INSTRUCTIONS_COMPLETE = False
        self.file_path = rospy.get_param('~instruction_filepath', '')
        if self.file_path == '':
            raise ValueError('No any file path for instruction file')
        self.readToList()
        print(self.instructions)

    def instruction_recieved(self, data):
        if data.data == "turnCompleted" and len(self.instructions) > 1:
            print("instruction executed")
            self.instructions.pop(0)
        else:
            self.INSTRUCTIONS_COMPLETE = True

    def readToList(self):
        with open(self.file_path) as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=',')
            for row in csv_reader:
                self.instructions.append(row)

    def publish_instruction(self):
        instruction = turn_instruction()
        if self.INSTRUCTIONS_COMPLETE:
            instruction.command = "stop"
            instruction.velocity = 0.0
        else:
            instruction.command = self.instructions[0][0]
            instruction.velocity = float(self.instructions[0][1])
        self.pub.publish(instruction)


if __name__ == '__main__':
    instruction_reader = ReadInstruction()
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        instruction_reader.publish_instruction()
        rate.sleep()
