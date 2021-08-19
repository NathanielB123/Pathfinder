from collections import namedtuple


class DynamicQueue:
    # Simple dynamic queue data structure
    def __init__(self):
        self.__queue_data = []
        self.__back_pointer = 0

    @property
    def __front_pointer(self):
        return len(self.__queue_data) - 1

    @property
    def empty(self):
        # When the front pointer +1 is equal to the back pointer, the queue is empty
        return not bool(self.__front_pointer - self.__back_pointer + 1)

    def enqueue(self, item):
        self.__queue_data.append(item)

    def dequeue(self):
        if not self.empty:
            self.__back_pointer += 1
            return self.__queue_data[self.__back_pointer - 1]
        else:
            raise Exception("Cannot pop from empty queue.")

    def peek(self):
        if not self.empty:
            return self.__queue_data[self.__front_pointer]
        else:
            raise Exception("Cannot peek empty queue.")

    def free_memory(self):
        # Call to remove all popped items from the queue data array; should be called every so often for large queues
        # to prevent memory leaks
        self.__queue_data = self.__queue_data[self.__back_pointer: self.__front_pointer + 1]
        self.__back_pointer = 0


class DynamicPriorityQueue:
    # Dynamic min-priority queue
    # Uses a binary heap to handle sorting of queue elements based on priority
    def __init__(self):
        self.__heap = DynamicBinaryHeap()
        pass

    @property
    def empty(self):
        # If the length of the heap is 0, the queue is empty
        return not bool(self.__heap.length)

    def enqueue(self, item, priority):
        self.__heap.insert_item(item, priority)

    def dequeue(self):
        return self.__heap.extract_root().item

    def peek(self):
        return self.__heap.root.item

    def decrease_priority_or_enqueue(self, item, new_priority):
        self.__heap.decrease_priority_or_insert(item, new_priority)


class DynamicBinaryHeap:
    # Dynamic binary min heap data structure
    # Heap contains tuples of the items are their respective priorities
    HeapItem = namedtuple("HeapItem", "item priority")

    def __init__(self):
        self.__heap_data = []

    @property
    def length(self):
        return len(self.__heap_data)

    @property
    def root(self):
        return self.__heap_data[0]

    def contains(self, item):
        for index in range(self.length):
            if self.__heap_data[index].item == item:
                return True
        return False

    @staticmethod
    def __parent_index(index):
        return (index - 1) // 2

    @staticmethod
    def __left_index(index):
        return index * 2 + 1

    @staticmethod
    def __right_index(index):
        return index * 2 + 2

    def __parent(self, index):
        return self.__heap_data[self.__parent_index(index)]

    def __left(self, index):
        return self.__heap_data[self.__left_index(index)]

    def __right(self, index):
        return self.__heap_data[self.__right_index(index)]

    def insert_item(self, item, priority):
        # Adds item to end of heap and up-heapifies to sort it into the correct position
        self.__heap_data.append(self.HeapItem(item, priority))
        self.__up_heapify(self.length - 1)

    def extract_root(self):
        root = self.root
        # Moves the bottom-right item in the heap to the root location
        self.__heap_data[0] = self.__heap_data[self.length - 1]
        self.__heap_data.pop(self.length - 1)
        # Sorts the heap to retain the heap property
        self.__down_heapify(0)
        return root

    def decrease_priority_or_insert(self, item, new_priority):
        index = self.__get_index(item)
        if index is None:
            self.insert_item(item, new_priority)
        else:
            self.__heap_data[index] = self.HeapItem(item, new_priority)
            self.__up_heapify(index)

    def __get_index(self, item):
        # Simple linear search for the requested item
        for index in range(self.length):
            if self.__heap_data[index].item == item:
                return index
        return None

    def __up_heapify(self, index):
        # Re-orders heap to make sure items of lower priority are above ones with higher
        # If the index of the root node (0) is reached, or if the child and parent do not need swapping,
        # the up-heapify operation is complete
        while not index == 0 and self.__heap_data[index].priority < self.__parent(index).priority:
            (self.__heap_data[index],
             self.__heap_data[self.__parent_index(index)]) = (self.__heap_data[self.__parent_index(index)],
                                                              self.__heap_data[index])
            index = self.__parent_index(index)

    def __down_heapify(self, index):
        # Re-orders heap to make sure items of higher priority are below ones with lower
        # If there are no children of the current item, the down-heapify operation is complete
        while self.__left_index(index) < self.length:
            min_index = index
            if self.__left(index).priority < self.__heap_data[min_index].priority:
                min_index = self.__left_index(index)
            if (self.__right_index(index) < self.length
                    and self.__right(index).priority < self.__heap_data[min_index].priority):
                min_index = self.__right_index(index)
            if min_index == index:
                # Children do not have lower priority values than the parent, so the down-heapify operation is complete
                break
            else:
                self.__heap_data[min_index], self.__heap_data[index] = (self.__heap_data[index],
                                                                        self.__heap_data[min_index])
                index = min_index
