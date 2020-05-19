import numpy as np

from proj1_2.code.Node import Node

class BinaryHeap:
    def __init__(self):
        self.items = []

    def size(self):
        return len(self.items)

    def getLeftChildIndex(self, parentIndex):
        return int(2 * parentIndex + 1)

    def getRightChildIndex(self, parentIndex):
        return int(2 * parentIndex + 2)

    def getParentIndex(self, childIndex):
        return int((childIndex - 1) / 2)




    def hasLeftChild(self, index):
        return self.getLeftChildIndex(index) < len(self.items)

    def hasRightChild(self, index):
        return self.getRightChildIndex(index) < len(self.items)

    def hasParent(self, index):
        return self.getParentIndex(index) >= 0




    def leftChild(self, index):
        return self.items[self.getLeftChildIndex(index)].k

    def rightChild(self, index):
        return self.items[self.getRightChildIndex(index)].k

    def parent(self, index):
        return self.items[self.getParentIndex(index)].k



    def swap(self, indexOne, indexTwo):
        temp = self.items[indexOne]
        self.items[indexOne] = self.items[indexTwo]
        self.items[indexTwo] = temp
        return

    # def peek(self):
    #     return self.items[0]

    def poll(self):
        if (len(self.items)) == 0:
            print("heap is empty for now, you may insert items...")
        item = self.items[0]
        self.items[0] = self.items[len(self.items) - 1]
        self.items.pop()
        self.heapifyDown()
        return item


    def insert(self, item:Node):
        self.items.append(item)
        self.heapifyUp()
        return

    def heapifyUp(self):
        index = len(self.items) - 1
        while self.hasParent(index) and (self.parent(index) > self.items[index].k):
            self.swap(self.getParentIndex(index), index)
            index = self.getParentIndex(index)
        return


    def heapifyDown(self):
        index = 0
        while (self.hasLeftChild(index)):
            smallerChildIndex = self.getLeftChildIndex(index)
            if self.hasRightChild(index) and self.rightChild(index) < self.leftChild(index):
                smallerChildIndex = self.getRightChildIndex(index)
            if self.items[index].k < self.items[smallerChildIndex].k:
                break
            else:
                self.swap(index, smallerChildIndex)
            index = smallerChildIndex
        return


    def contains(self, item:Node):
        if item in self.items:
            return True
        return False


