import scipy.io
import sys
import numpy as np
import matplotlib.pyplot as plt

class PathPlanning:
    def __init__(self):
        sys.setrecursionlimit(5000)
        maps  = scipy.io.loadmat("maze.mat")
        map = maps['map']
        map = np.array(map, dtype=np.uint32)
        start_row, start_col = self.printWelcomeMessages()
        while (start_row > map.shape[0]) or (start_col > map.shape[1]) or (map[start_row][start_col] == 1) or (start_row <= 0) or (start_col <= 0):
            start_row, start_col = self.printRedoMessages()
        [trajectory, value_map] = self.planner(map, start_row, start_col) #Main runner function
        print("Filling table...")
        # print("value_map = ")
        # self.printArray(value_map)
        print("Planning trajectory...")
        print("trajectory = ")
        self.printArray(trajectory)
        self.drawTrajectory(trajectory, map, start_row, start_col)
    
    def planner(self, map, start_row, start_col):
        for i in range(len(map)):
            for j in range(len(map[0])):
                if map[i][j] == 2:
                    self.row_stop = i
                    self.col_stop = j
        value_map = self.fillMap(map, [[self.row_stop, self.col_stop]])
        self.trajectory = [[start_row + 1, start_col + 1]]
        self.trajectory = self.planTrajectory(map, start_row, start_col)

        return [self.trajectory, value_map]

    def printArray(self, arr):
        for row in arr:
            print(row)

    def printWelcomeMessages(self):
        print("Welcome to our program!")
        print("Now let's start by entering our starting position!")
        in1 = int(input('Enter start row position: '))
        in2 = int(input('Enter start column position: '))
        return in1 - 1, in2 - 1

    def printRedoMessages(self):
        print("The starting points you entered occur outside of boundaries or are on an obstacle, please re-enter!")
        in1 = int(input('Enter start row position: '))
        in2 = int(input('Enter start column position: '))
        return in1 - 1, in2 - 1

    def fillMap(self, map, points):
        if (points):
            new_points = []
            for i in range(len(points)):
                row = points[i][0]
                col = points[i][1]
                if(map[row - 1][col] == 0):
                    map[row - 1][col] = map[row][col] + 1 #Upper
                    new_points.append([row - 1, col])
                if(map[row][col + 1] == 0):
                    map[row][col + 1] = map[row][col] + 1 #Right
                    new_points.append([row, col + 1])
                if(map[row + 1][col] == 0):
                    map[row + 1][col] = map[row][col] + 1 #Lower
                    new_points.append([row + 1, col])
                if(map[row][col - 1] == 0):
                    map[row][col - 1] = map[row][col] + 1 #Left
                    new_points.append([row, col - 1])
                
                if(map[row - 1][col + 1] == 0):
                    map[row - 1][col + 1] = map[row][col] + 1 #Upper Right
                    new_points.append([row - 1, col + 1])
                if(map[row + 1][col + 1] == 0):
                    map[row + 1][col + 1] = map[row][col] + 1 #Lower Right
                    new_points.append([row + 1, col + 1])
                if(map[row + 1][col - 1] == 0):
                    map[row + 1][col - 1] = map[row][col] + 1 #Lower Left
                    new_points.append([row + 1, col - 1])
                if(map[row - 1][col - 1] == 0):
                    map[row - 1][col - 1] = map[row][col] + 1 #Upper Left 
                    new_points.append([row - 1, col - 1])
            self.fillMap(map, new_points)
        return map
        
    def planTrajectory(self, map, start_row, start_col):
        if(map[start_row - 1][start_col] < map[start_row][start_col] and map[start_row - 1][start_col] != 1): #Upper
            self.trajectory.append([start_row - 1 + 1, start_col + 1])
            self.planTrajectory(map, start_row - 1, start_col)
        elif(map[start_row][start_col + 1] < map[start_row][start_col] and map[start_row][start_col + 1] != 1): #Right
            self.trajectory.append([start_row + 1, start_col + 1 + 1])
            self.planTrajectory(map, start_row, start_col + 1)
        elif(map[start_row + 1][start_col] < map[start_row][start_col] and map[start_row + 1][start_col] != 1): #Lower
            self.trajectory.append([start_row + 1 + 1, start_col + 1])
            self.planTrajectory(map, start_row + 1, start_col)
        elif(map[start_row][start_col - 1] < map[start_row][start_col] and map[start_row][start_col - 1] != 1): #Left
            self.trajectory.append([start_row + 1, start_col - 1 + 1])
            self.planTrajectory(map, start_row, start_col - 1)

        elif(map[start_row - 1][start_col + 1] < map[start_row][start_col] and map[start_row - 1][start_col + 1] != 1): #Upper Right
            self.trajectory.append([start_row - 1 + 1, start_col + 1 + 1])
            self.planTrajectory(map, start_row - 1, start_col + 1)
        elif(map[start_row + 1][start_col + 1] < map[start_row][start_col] and map[start_row + 1][start_col + 1] != 1): #Lower Right
            self.trajectory.append([start_row + 1 + 1, start_col + 1 + 1])
            self.planTrajectory(map, start_row + 1, start_col + 1)
        elif(map[start_row + 1][start_col - 1] < map[start_row][start_col] and map[start_row + 1][start_col - 1] != 1): #Lower Left
            self.trajectory.append([start_row + 1 + 1, start_col - 1 + 1])
            self.planTrajectory(map, start_row + 1, start_col - 1)
        elif(map[start_row - 1][start_col - 1] < map[start_row][start_col] and map[start_row - 1][start_col - 1] != 1): #Upper Left
            self.trajectory.append([start_row - 1 + 1, start_col - 1 + 1])
            self.planTrajectory(map, start_row - 1, start_col - 1)
        else:
            if(start_row == self.row_stop) and (start_col == self.col_stop):
                print('Target Reached!')
            else:
                print('No trajectory found!')
        return self.trajectory

    def drawTrajectory(self, trajectory, map, start_row, start_col):
        for i in trajectory:
            map[i[0] - 1][i[1] - 1] = 0 #Red
        for i in range(len(map)):
            for j in range(len(map[0])):
                if map[i][j] == 1:
                    map[i][j] = 255 #Blue
                elif map[i][j] == 0:
                    pass
                else:
                    map[i][j] = 150 #White-ish
        map[self.row_stop][self.col_stop] =  200 #Light Blue
        map[start_row][start_col] =  100 #Light Red
        plt.imshow(map, cmap = 'RdYlBu')
        plt.show()
        er = input("Enter 0 to exit, 1 to restart again: ")
        if er == '0':
            sys.exit('Program ended by user command.')
        elif er == '1':
            self.__init__()


if __name__ == '__main__':
    program = PathPlanning()