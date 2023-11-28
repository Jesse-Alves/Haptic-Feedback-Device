from random import shuffle, randrange
from numpy import array
import numpy
import os

def make_maze(w, h):
    vis = [[0] * w + [1] for _ in range(h)] + [[1] * (w + 1)]
    ver = [["+  "] * w + ['+'] for _ in range(h)] + [[]]
    hor = [["+--"] * w + ['+'] for _ in range(h + 1)]

    def walk(x, y):
        vis[y][x] = 1

        d = [(x - 1, y), (x, y + 1), (x + 1, y), (x, y - 1)]
        shuffle(d)
        for (xx, yy) in d:
            if vis[yy][xx]: continue
            if xx == x: hor[max(y, yy)][x] = "+  "
            if yy == y: ver[y][max(x, xx)] = "   "
            walk(xx, yy)

    walk(randrange(w), randrange(h))

    s = ""
    for (a, b) in zip(hor, ver):
        s += ''.join(a + ['\n'] + b + ['\n'])
    return s

def convert_maze(maze_raw, w, h, workspace):
    maze_numeric = []

    # Set the radius size for the spheres
    qtd_spheres_w = 3*w+1
    qtd_spheres_h = 2*h+1

    w_num = abs(workspace[0][0] - workspace[1][0])
    h_num = abs(workspace[0][1] - workspace[2][1])

    radius = (w_num/qtd_spheres_w)/2.0
    radius_h = (h_num/qtd_spheres_h)/2.0

    #print('Element by Element:')
    x = workspace[0][0]
    y = workspace[0][1]
    cont = 0
    for i in range(qtd_spheres_h):        
        for j in range(qtd_spheres_w+1):
            if (maze_raw[cont] == '+') or (maze_raw[cont] == '-'): 
                #print(j)              
                if (j == 0) or (j == qtd_spheres_w-1):
                    maze_numeric.append((1.5*3*radius,x, y))
                else: 
                    maze_numeric.append((3*radius,x, y))
                #print(maze_raw[j])
            #print(f' Element {cont}: {maze_raw[cont]}')
            cont = cont + 1
            x = x - 2*radius            
        x = workspace[0][0]
        y = y + 2*radius_h        

    '''
    # Take each element of the maze wall    
    for i in range(qtd_spheres_h*qtd_spheres_w):            
        
        if maze_raw[i] == '+' or maze_raw[i] == '-':
            print(maze_raw[i])'''


    return maze_numeric

if __name__ == '__main__':

    #Maze Size
    w = 6
    h = 3

    # Generate a random Maze
    maze_raw = make_maze(w,h)
    print(maze_raw)

    ''' ===================== Informations =======================================
    -> It is necessary to convert the Raw Maze into a Maze with center of spheres
    -> Each maze wall element should be a point with respect to the origin 
    -> It is necessary to pass the robot workspace and the radius of the spheres
    -> The workspace here is defined as the 4 corner points
    =============================================================================='''

    # WorkSpace Information
    workspace = array([[0.025, 0.09404],[-0.075, 0.09404],[0.025, 0.16845],[-0.075, 0.16845]])
    

    maze_numeric = numpy.array(convert_maze(maze_raw, w, h, workspace))     

    # Export csv file.
    #pd.DataFrame(np_array).to_csv("path/to/file.csv")
    numpy.savetxt("/home/telecom/haptic_ws/src/haptic_nodes/scripts/maze1_LowCost.csv", maze_numeric, delimiter=",")
    print('Maze .csv File exported')
