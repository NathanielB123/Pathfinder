class DynamicStack:
    # Simple dynamic stack data structure
    def __init__(self):
        self.__stack_data = []

    @property
    def __stack_pointer(self):
        return len(self.__stack_data) - 1

    @property
    def empty(self):
        # When the stack pointer is equal to -1, the stack is empty
        return not bool(self.__stack_pointer + 1)

    def push(self, item):
        self.__stack_data.append(item)

    def pop(self):
        if not self.empty:
            return self.__stack_data.pop()
        else:
            raise Exception("Cannot pop from empty stack.")

    def peek(self):
        if not self.empty:
            return self.__stack_data[self.__stack_pointer]
        else:
            raise Exception("Cannot peek empty stack.")
