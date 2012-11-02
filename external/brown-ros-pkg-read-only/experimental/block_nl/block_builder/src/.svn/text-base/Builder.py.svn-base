import rospy

import rospy
from pr2_block_builder_msgs.msg import *
from Picker import *
from Placer import *

class Builder():

    ready = True
    picker = None
    placer = None
    status_publisher = None
    block_status_publisher = None

    def __init__(self):
        self.status_publisher = rospy.Publisher('/pr2_block_builder/status', Status, latch=True)
        self.block_status_publisher = rospy.Publisher('/pr2_block_builder/completion', BlockStatus)
        self.picker = Picker()
        self.placer = Placer()
        rospy.sleep(1)
        self.sendStatus("ready")

    def start(self, blocks):
        if (self.ready):
            # Prevent further invocations
            self.ready = False

            # Send the start status
            self.sendStatus("started")

            # Sort the blocks
            self.sortBlocks(blocks, 0)

            # Do construction
            status = self.construct(blocks)

            # Send the completion status
            self.sendStatus(status)
    
    def stop(self):
        self.ready = False
        if self.picker!=None:
            self.picker.cancel()
        if self.placer!=None:
            self.placer.cancel()

    def restart(self):
        self.stop()
        self.ready = True
        self.sendStatus("ready")

    def construct(self, blocks):

        for i in range(0, len(blocks)):
            block = blocks[i]

            # Send the status
            self.sendBlockStatus(block, "started")
            
            # Pick
            status = self.picker.pick(i)
            
            # Place
            if status=="completed":
                status = self.placer.place(block, false)

            # Send the status
            self.sendBlockStatus(block, status);

            # Stop if we've failed or aborted
            if status!="completed":
                return status
            
        return "completed"

    def sendStatus(self, status, message=""):
        rospy.loginfo("Status: %s", status)

        statusObject = Status()
        statusObject.status.data = status
        statusObject.message.data = message

        self.status_publisher.publish(statusObject)

    def sendBlockStatus(self, block, status, message=""):
        rospy.loginfo("Block (%s, %s, %s) %s", block.position.x, block.position.y, block.position.z, status)

        statusObject = BlockStatus()
        statusObject.block = block
        statusObject.status.data = status
        statusObject.message.data = message

        self.block_status_publisher.publish(statusObject)

    def sortBlocks(self, blocks, start):
        dist_block = blocks[i].position.x * blocks[i].position.x + blocks[i].position.y * blocks[i].position.y
        nextBlock = blocks[start]
        nextBlockIndex = start
        dist_nextBlock = dist_block
        for i in range(start, len(blocks)):
            dist_block = blocks[i].position.x * blocks[i].position.x + blocks[i].position.y * blocks[i].position.y
            if (blocks[i].position.z < nextBlock.position.z):
                nextBlock = blocks[i]
                nextBlockIndex = i
                dist_nextBlock = dist_block
            elif (blocks[i].position.z == nextBlock.position.z and dist_block < dist_nextBlock):
                nextBlock = blocks[i]
                nextBlockIndex = i
                dist_nextBlock = dist_block
            elif (blocks[i].position.z == nextBlock.position.z and dist_block == dist_nextBlock and blocks[i].position.x < nextBlock.position.x):
                nextBlock = blocks[i]
                nextBlockIndex = i
                dist_nextBlock = dist_block
        blocks[nextBlockIndex] = blocks[start]
        blocks[start] = nextBlock
        if (start == len(blocks)-1):
            return blocks
        return self.sortBlocks(blocks, start + 1)

