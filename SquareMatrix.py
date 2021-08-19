class SquareMat:
    # Matrix of values, with equal width and height
    def __init__(self, size, default_val=0):
        # Matrix is initialised populated with default_val
        self.__matrix_data = [[default_val for _ in range(size)] for _ in range(size)]

    @property
    def size(self):
        return len(self.__matrix_data)

    def get_item(self, x, y):
        return self.__matrix_data[x][y]

    def set_item(self, x, y, num, mirrored=True):
        self.__matrix_data[x][y] = num
        if mirrored:
            self.__matrix_data[y][x] = num

    def expand(self, default_val=0):
        for row_num in range(self.size):
            self.__matrix_data[row_num].append(default_val)
        self.__matrix_data.append([default_val for _ in range(self.size+1)])

    def delete_row_column(self, row_col_num):
        self.__matrix_data.pop(row_col_num)
        for row_num in range(self.size):
            self.__matrix_data[row_num].pop(row_col_num)
