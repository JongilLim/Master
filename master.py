# Standard set for Ice
import sys, traceback#, Ice
import time
import time
import pika
import uuid
from math import *
import random
import csv
import lineutils

# some top level parameters
eps = 1.0e-10

# grid map
grid = [[0, 1, 0, 0, 1, 1, 1],
        [0, 1, 0, 0, 1, 0, 0],
        [0, 0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 1],
        [1, 1, 1, 1, 0, 0, 1],
        [0, 0, 1, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 1, 0],
        [0, 0, 0, 0, 0, 1, 1],
        [1, 1, 0, 1, 0, 0, 0],
        [0, 0, 0, 1, 0, 1, 1],
        [0, 0, 0, 1, 0, 0, 0]]

# the number of sub PCs
num_PC = 0

#x,y coordinate
coordinate = [[0 for row in range(len(grid[0]))] for col in range(len(grid))]

init = [] # start point
goal = [] # goal point
path = []

delta = [[-1, 0 ], # go up
         [ 0, -1], # go left
         [ 1, 0 ], # go down
         [ 0, 1 ]] # go right

cost = 1

# Measurements could be inprecise

bearing_noise = 0.3 # Noise parameter: should be included in sense function.
steering_noise = 0.1 # Noise parameter: should be included in move function.
distance_noise = 0.1 # Noise parameter: should be included in move function.

# List of wall segments [((start_x,start_y),(end_x,end_y)), (...)]
room_plan = []

# Horizontal (x) and vertical (y) ranges of the room
world_x_range = [0.0, 0.0]
world_y_range = [0.0, 0.0]

# Robot dimensions in meters
robot_length = 0.320
robot_width = 0.180

#connect rabbitMQ server to communicate smartphone
userinfo = pika.PlainCredentials('bbb','123123') # server ID : bbb, password : 123123
connection = pika.BlockingConnection(pika.ConnectionParameters('172.21.11.242',credentials=userinfo)) # localhost : 172.21.11.242

#create channel for rabbitMQ
channel1 = None 
channel2 = None

#connect channel
channel1 = connection.channel()
channel2 = connection.channel()

#body to receive weight and particles from sever
w_and_p = None


#make heuristic from grid map
def make_heuristic(grid, goal, cost):
    
    heuristic = [[0 for row in range(len(grid[0]))] 
                      for col in range(len(grid))]
    for i in range(len(grid)):    
        for j in range(len(grid[0])):
            heuristic[i][j] = abs(i - goal[0]) + abs(j - goal[1])

    return heuristic


#create x and y coordinate from grid map. x and y has interval with 33.5cm
def make_coordinate(grid):

    x = 0.335
    y = 0.67*len(grid) - 0.335

    for i in range(len(grid)):
        for j in range(len(grid[0])):
            coordinate[i][j] = [round(x,3), round(y,3)]
            x += 0.67
        y -= 0.67
        x= 0.335

    return coordinate


#create path from start point to goal point
def search(init, goal, position):

    heuristic = make_heuristic(grid, goal, cost)
    
    closed = [[0 for row in range(len(grid[0]))] for col in range(len(grid))]
    closed[init[0]][init[1]] = 1
    action = [[-1 for row in range(len(grid[0]))] for col in range(len(grid))]

    x = init[0]
    y = init[1]
    g = 0
    h = heuristic[x][y]
    f = g + h

    open_map = [[f, g, h, x, y]]

    found = False  # flag that is set when search is complet
    resign = False # flag set if we can't find expand

    #path plan by A* algorithm
    while not found and not resign:
        if len(open_map) == 0:
            resign = True
            return 'fail'
        else:
            open_map.sort()
            open_map.reverse()
            next = open_map.pop()
            x = next[3]
            y = next[4]
            g = next[1]
            
            if x == goal[0] and y == goal[1]:
                found = True
            else:
                for i in range(len(delta)):
                    x2 = x + delta[i][0]
                    y2 = y + delta[i][1]
                    if x2 >= 0 and x2 < len(grid) and y2 >=0 and y2 < len(grid[0]):
                        if closed[x2][y2] == 0 and grid[x2][y2] == 0:
                            g2 = g + cost
                            h2 = heuristic[x2][y2]
                            f2 = g2 + h2

                            open_map.append([f2, g2, h2, x2, y2])

                            closed[x2][y2] = 1
                            action[x2][y2] = i
                                
    x = goal[0]
    y = goal[1]
    path = [goal]
    while x != init[0] or y != init[1]:    
        x2 = x - delta[action[x][y]][0]
        y2 = y - delta[action[x][y]][1]
        x = x2
        y = y2
        path.append((x,y))
        
    path.reverse()

    robot_coordinate = []

    #change grid coordinate to real coordinate
    for i in range(len(path)):
        x = path[i][0]
        y = path[i][1]
        robot_coordinate.append(coordinate[x][y])

    if position != None:
        robot_coordinate[0] = (position[0],position[1])

    #chagne path to smooth path
    robot_coordinate = smooth(robot_coordinate)

    #save path
    f = open('trajectory'+'.dat', 'w')
    for i in range(len(robot_coordinate)):
        f.write (str(robot_coordinate[i][0])+'\t'+str(robot_coordinate[i][1])+'\n')
    f.close()

    return path


#change path to smooth path
def smooth(path, weight_data = 0.5, weight_smooth = 0.1, tolerance = 0.000001):

    newpath = [[0 for row in range(len(path[0]))] for col in range(len(path))]
    for i in range(len(path)):
        for j in range(len(path[0])):
            newpath[i][j] = path[i][j]
            
    change = tolerance
    while change >= tolerance:
        change = 0.0
        for i in range(1, len(path)-1):
            for j in range(len(path[0])):
                aux = newpath[i][j]
                newpath[i][j] += weight_data * (path[i][j] - newpath[i][j])
                newpath[i][j] += weight_smooth * (newpath[i-1][j] + newpath[i+1][j] - (2.0 * newpath[i][j]))
                change += abs(aux - newpath[i][j])
    
    return newpath

    
# Generates motions list based on the trajectory specified as a set of
# waypoints. We assume, that initial position of the robot has
# coordinates as specified in the first line of the trajectory file
# and bearing (angle) is 0, i.e. robot is parallel to X axis facing to
# the right.
def generate_motions(trajectory_file_name, prev_angle):
    
    motions = []
    f = open(trajectory_file_name, 'r')

    prev_angle = 0.0
    b = None
    for line in f:
        x, y = line.split('\t')
        if b is not None:
            a = b
            b = (float(x), float(y))
            (x1, y1) = a
            (x2, y2) = b
            (x0, y0) = ((x2 - x1), (y2 - y1))

            if x0 > 0 and y0 >= 0:
                theta = atan(y0/x0)
            elif x0 > 0 and y0 < 0:
                theta = atan(y0/x0) + 2 * pi
            elif x0 < 0:
                theta = atan(y0/x0) + pi
            elif x0 == 0 and y0 > 0:
                theta = pi/2
            elif x0 == 0 and y0 <0:
                theta = (3*pi)/2

            distance = sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1))

            rotate = prev_angle - theta 
            if 0 < rotate and rotate < pi:
                rotate = -abs(rotate)
            elif -pi < rotate and rotate < 0:
                rotate = abs(rotate)
            elif pi < rotate:
                rotate = (2*pi) - abs(rotate)
            elif rotate < -pi:
                rotate = -((2*pi) - abs(rotate))

            motions.append([rotate,distance,[x2,y2,theta]])
            prev_angle = theta

        else:
            b = (float(x), float(y))

    f.close()
    
    return motions


# Request left and right wheel to travel sl and sr distance
# speed = +1 foreward, speed = -1 backward
def make_motion_step(sl, sr, speed = 1):

    if abs(sl) < eps:
        actuator_cmd1_speed = 0
        actuator_cmd1_distance = 0
    else:
        actuator_cmd1_speed = speed if sl >= 0 else -speed
        actuator_cmd1_distance = abs(sl) / (3.14159 * 0.06)

    if abs(sr) < eps:
        actuator_cmd2_speed = 0
        actuator_cmd2_distance = 0
    else:
        actuator_cmd2_speed = speed if sr >= 0 else -speed
        actuator_cmd2_distance = abs(sr) / (3.14159 * 0.06)

    movement = [1, actuator_cmd1_speed, round(actuator_cmd1_distance,2), actuator_cmd2_speed, round(actuator_cmd2_distance,2)]
    move = ' '.join(str(n) for n in movement)+' '+'\n'

    return move
    

#check obstacle
def check_obstacle(theta):
    
    if theta < 0:
        theta += 2*pi
    degrees = int(round((180 * theta / pi),0))
    
    if 0 <= degrees and degrees < 90:
        rotate = [1, degrees]
    elif 90 <= degrees and degrees < 180:
        degrees -= 90
        rotate = [2, degrees]
    elif 180 <= degrees and degrees < 270:
        degrees -= 180
        rotate = [3, degrees]
    elif 270 <= degrees and degrees < 360:
        degrees -= 270
        rotate = [4, degrees]

    #2014/11/20
    rotation = '2'+' '+' '.join(str(n) for n in rotate)+' '+'\n'

    return rotation


def measurement_prob(plan_list, particle, measurements):
    
    # exclude particles outside the room
    if not lineutils.point_inside_polygon(plan_list, particle):
        return 0.0

    # calculate the correct measurement
    predicted_measurements = lineutils.measurements(room_plan, particle)

    # compute errors
    prob = 1.0
    count = 0
    for i in xrange(0, len(measurements)):
        if measurements[i] != 0:
            error_mes = abs(measurements[i] - predicted_measurements[i])
            # update Gaussian
            #error *= (exp(- (error_mes ** 2) / (bearing_noise ** 2) / 2.0) / sqrt(2.0 * pi * (bearing_noise ** 2)))

            #8/28
            prob += (exp(- (error_mes ** 2) / (bearing_noise ** 2) / 2.0) / sqrt(2.0 * pi * (bearing_noise ** 2)))
            count += 1
    prob /= count
    prob = prob ** 4

    return prob


# Here we are using equations for two-wheels differential
# steering system as presented here 
# http://rossum.sourceforge.net/papers/DiffSteer/DiffSteer.html :
#
# S = (Sr+Sl)/2
# Theta = (Sr-Sl)/2b+theta0
# X = s*cos(theta)+x0
# Y = s*sin(theta)+y0
# Where Sr and Sl is the distance travelled by each wheel and b is the
# distance between wheels (vehicle width)
#
def move(particle, motion):
    
    (x,y,theta0) = particle
    (delta_theta, s, _) = motion
    delta_theta = random.gauss(delta_theta, steering_noise)
    s = random.gauss(s, distance_noise)
    
    theta = theta0 + delta_theta;
    x += s * cos(theta)
    y += s * sin(theta)

    return (x,y,theta)


# extract position from a particle set
def get_position(p):
    
    x = 0.0
    y = 0.0
    orientation = 0.0
    X = []
    Y = []
    D = []
    (_,_,init_orientation) = p[0]
    for (px,py,theta) in p:
        x += px
        y += py

        X.append(px)
        Y.append(py)
        # orientation is tricky because it is cyclic. By normalizing
        # around the first particle we are somewhat more robust to
        # the 0=2pi problem
        orientation += (((theta - init_orientation + pi) % (2.0 * pi)) 
                        + init_orientation - pi)
        D.append((((theta - init_orientation + pi) % (2.0 * pi)) 
                        + init_orientation - pi))

    #Average and variance of x, y and d respectively
    average_x = x / len(p)
    variance_x = sum((average_x - value ) ** 2 for value in X) / len(X)
    average_y = y / len(p)
    variance_y = sum((average_y - value ) ** 2 for value in Y) / len(Y)
    average_d = orientation / len(p)
    variance_d = sum((average_d - value ) ** 2 for value in D) / len(D)

    return (x / len(p), y / len(p), orientation / len(p), variance_x, variance_y, variance_d)


#calculate rotate, when the mobile robot adjusts position.
def calculate_rotate(rotate):
    
    if 0 < rotate and rotate < pi:
        rotate = -abs(rotate)
    elif -pi < rotate and rotate < 0:
        rotate = abs(rotate)
    elif pi < rotate:
        rotate = (2*pi) - abs(rotate)
    elif rotate < -pi:
        rotate = -((2*pi) - abs(rotate))

    return rotate


#mobile robot move to correct position, when the mobile robot adjusts position.
def move_position(x1, y1, prev_angle, x2, y2, original_d):

    (x0, y0) = ((x2 - x1), (y2 - y1))

    if x0 > 0 and y0 >= 0:
        theta = atan(y0/x0)
    elif x0 > 0 and y0 < 0:
        theta = atan(y0/x0) + 2 * pi
    elif x0 < 0:
        theta = atan(y0/x0) + pi
    elif x0 == 0 and y0 > 0:
        theta = pi/2
    elif x0 == 0 and y0 <0:
        theta = (3*pi)/2

    distance = sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1))

    rotate = calculate_rotate(prev_angle - theta ) 
    adjust_direction = calculate_rotate(theta -  original_d)

    rpc = FibonacciRpcClient()
    
    sr = rotate * robot_width
    sl = -sr

    M = make_motion_step(sl, sr) # Rotate on place
    a = rpc.call(M)
    print a #meassage  "Drive done"
    M = make_motion_step(distance, distance) # Drive forward
    b = rpc.call(M)
    print b #meassage "Drive done"
    
    sr = adjust_direction * robot_width
    sl = -sr
    
    M = make_motion_step(sl, sr) # Rotate on place
    c = rpc.call(M)
    print c #meassage "Drive done"
    

#repath planning
def repath(path, path_count, goal, position):

    #2014/09/25
    coordinate = [position[0],position[1]]
    
    print "Obstacle : ", path[path_count+1]
    obstacle = path[path_count+1]
    x = obstacle[0]
    y = obstacle[1]
    grid[x][y] = 1
    for i in range(len(grid)):
        print grid[i]

    path = search(path[path_count], goal, coordinate)
    
    grid[x][y] = 0

    return path


#2014/11/12
def callback(ch, method, properties, body):
    global channel2
    global w_and_p
    w_and_p = body
    ch.basic_ack(delivery_tag = method.delivery_tag)

    channel2.stop_consuming()


#prediction and measurement update
def prediction_and_measurement(motions, path, goal, N, p, iter_count, complete, position):
    global channel1
    global channel2
    global weight


    path_count = 0
    
    for motion in motions:
	
        fibonacci_rpc = FibonacciRpcClient()

        (delta_theta, s, _) = motion
        
        distance1 = check_obstacle(delta_theta)
		
	#2014/11/20
        distance = fibonacci_rpc.call_1(distance1)
        distance = float(distance)/100
        print "check distance : ", distance

        #check obstacle
        if 0.02 < distance and distance < s + 0.02:
            print "Path replanning"

            path = repath(path, path_count, goal, position[path_count-1])
            motions = generate_motions('trajectory.dat', position[path_count-1][2])

            for i in range(len(motions)):
                position.append(motions[i][2])
                
            complete = 1

            return (motions, path, p, iter_count, complete, position)

        if num_PC == 2:
            #devide particle to 2 parts for sending particles to 2 computers respectively
            p_1 = p[:len(p)/2]
            p_2 = p[len(p)/2:]
            str_delta_theta = str(delta_theta)
            str_s = str(s)
            p1_str = ' '.join(' '.join(map(str,l)) for l in p_1)
            p2_str = ' '.join(' '.join(map(str,l)) for l in p_2)
            #add theta and s for movement update
            p1_str = p1_str + ' ' + str_delta_theta + ' ' + str_s
            p2_str = p2_str + ' ' + str_delta_theta + ' ' + str_s

        #uncomment when use 4 computers
        elif num_PC == 4:
            p_1 = p[:len(p)/2]
            part_1 = p_1[:len(p_1)/2]
            part_2 = p_1[len(p_1)/2:]
            
            p_2 = p[len(p)/2:]
            part_3 = p_2[:len(p_2)/2]
            part_4 = p_2[len(p_2)/2:]
            
            str_delta_theta = str(delta_theta)
            str_s = str(s)
            
            p1_str = ' '.join(' '.join(map(str,l)) for l in part_1)
            p2_str = ' '.join(' '.join(map(str,l)) for l in part_2)
            p3_str = ' '.join(' '.join(map(str,l)) for l in part_3)
            p4_str = ' '.join(' '.join(map(str,l)) for l in part_4)
            
            p1_str = p1_str + ' ' + str_delta_theta + ' ' + str_s
            p2_str = p2_str + ' ' + str_delta_theta + ' ' + str_s
            p3_str = p3_str + ' ' + str_delta_theta + ' ' + str_s
            p4_str = p4_str + ' ' + str_delta_theta + ' ' + str_s

        #uncomment when use 8 computers
        elif num_PC == 8:
            p_1 = p[:len(p)/2]
            
            part_1 = p_1[:len(p_1)/2]
            newP_1 = part_1[:len(part_1)/2]
            newP_2 = part_1[len(part_1)/2:]
            
            part_2 = p_1[len(p_1)/2:]
            newP_3 = part_2[:len(part_2)/2]
            newP_4 = part_2[len(part_2)/2:]
            
            p_2 = p[len(p)/2:]
            
            part_3 = p_2[:len(p_2)/2]
            newP_5 = part_3[:len(part_3)/2]
            newP_6 = part_3[len(part_3)/2:]
            
            part_4 = p_2[len(p_2)/2:]
            newP_7 = part_4[:len(part_4)/2]
            newP_8 = part_4[len(part_4)/2:]
                
            str_delta_theta = str(delta_theta)
            str_s = str(s)
            
            p1_str = ' '.join(' '.join(map(str,l)) for l in newP_1)
            p2_str = ' '.join(' '.join(map(str,l)) for l in newP_2)
            p3_str = ' '.join(' '.join(map(str,l)) for l in newP_3)
            p4_str = ' '.join(' '.join(map(str,l)) for l in newP_4)
            p5_str = ' '.join(' '.join(map(str,l)) for l in newP_5)
            p6_str = ' '.join(' '.join(map(str,l)) for l in newP_6)
            p7_str = ' '.join(' '.join(map(str,l)) for l in newP_7)
            p8_str = ' '.join(' '.join(map(str,l)) for l in newP_8)
            
            p1_str = p1_str + ' ' + str_delta_theta + ' ' + str_s
            p2_str = p2_str + ' ' + str_delta_theta + ' ' + str_s
            p3_str = p3_str + ' ' + str_delta_theta + ' ' + str_s
            p4_str = p4_str + ' ' + str_delta_theta + ' ' + str_s
            p5_str = p5_str + ' ' + str_delta_theta + ' ' + str_s
            p6_str = p6_str + ' ' + str_delta_theta + ' ' + str_s
            p7_str = p7_str + ' ' + str_delta_theta + ' ' + str_s
            p8_str = p8_str + ' ' + str_delta_theta + ' ' + str_s

        #when use 1 computer
        else:
            print("Skip")
        

        #send divided particles with movement values to each computer
        channel1.queue_declare(queue='particle', durable=True)

        if num_PC == 2:
            channel1.basic_publish(exchange='',
                          routing_key='particle',
                          body=p1_str,
                          properties=pika.BasicProperties(
                             delivery_mode = 2, # make message persistent
                          ))
            channel1.basic_publish(exchange='',
                          routing_key='particle',
                          body=p2_str,
                          properties=pika.BasicProperties(
                             delivery_mode = 2, # make message persistent
                          ))
            print 'Done sending particles'

        elif num_PC == 4:
            channel1.basic_publish(exchange='',
                          routing_key='particle',
                          body=p1_str,
                          properties=pika.BasicProperties(
                             delivery_mode = 2, # make message persistent
                          ))
            channel1.basic_publish(exchange='',
                          routing_key='particle',
                          body=p2_str,
                          properties=pika.BasicProperties(
                             delivery_mode = 2, # make message persistent
                          ))
            channel1.basic_publish(exchange='',
                          routing_key='particle',
                          body=p1_str,
                          properties=pika.BasicProperties(
                             delivery_mode = 2, # make message persistent
                          ))
            channel1.basic_publish(exchange='',
                          routing_key='particle',
                          body=p2_str,
                          properties=pika.BasicProperties(
                             delivery_mode = 2, # make message persistent
                          ))
            print 'Done sending particles'

        elif num_PC == 8:
            channel1.basic_publish(exchange='',
                          routing_key='particle',
                          body=p1_str,
                          properties=pika.BasicProperties(
                             delivery_mode = 2, # make message persistent
                          ))
            channel1.basic_publish(exchange='',
                          routing_key='particle',
                          body=p2_str,
                          properties=pika.BasicProperties(
                             delivery_mode = 2, # make message persistent
                          ))
            channel1.basic_publish(exchange='',
                          routing_key='particle',
                          body=p1_str,
                          properties=pika.BasicProperties(
                             delivery_mode = 2, # make message persistent
                          ))
            channel1.basic_publish(exchange='',
                          routing_key='particle',
                          body=p2_str,
                          properties=pika.BasicProperties(
                             delivery_mode = 2, # make message persistent
                          ))
            channel1.basic_publish(exchange='',
                          routing_key='particle',
                          body=p1_str,
                          properties=pika.BasicProperties(
                             delivery_mode = 2, # make message persistent
                          ))
            channel1.basic_publish(exchange='',
                          routing_key='particle',
                          body=p2_str,
                          properties=pika.BasicProperties(
                             delivery_mode = 2, # make message persistent
                          ))
            channel1.basic_publish(exchange='',
                          routing_key='particle',
                          body=p1_str,
                          properties=pika.BasicProperties(
                             delivery_mode = 2, # make message persistent
                          ))
            channel1.basic_publish(exchange='',
                          routing_key='particle',
                          body=p2_str,
                          properties=pika.BasicProperties(
                             delivery_mode = 2, # make message persistent
                          ))
            print 'Done sending particles'
        else:
            p = map(lambda particle: move(particle, motion), p)
        
        sr = (delta_theta) * robot_width
        sl = -sr   

        M = make_motion_step(sl, sr) # Rotate on place
        a = fibonacci_rpc.call(M)
        print a
        M = make_motion_step(s, s) # Drive forward
        b = fibonacci_rpc.call(M)
        print b
        
        print "Waitng measurment from smart phone"
        # Measurement update
        Z = fibonacci_rpc.call("0")
        Z = Z.split(" ")
        del Z[12]
        Z = map(float, Z)
        for i in range(0,12):
            Z[i] = float(Z[i])/100
            if Z[i] != 0:
                Z[i] += 0.055
        Z_str = ' '.join(str(n) for n in Z)

        #send measurement to each computer
        channel1.queue_declare(queue='measurement', durable=True)

        if num_PC == 2:
            channel1.basic_publish(exchange='',
                          routing_key='measurement',
                          body=Z_str,
                          properties=pika.BasicProperties(
                             delivery_mode = 2, # make message persistent
                          ))
            channel1.basic_publish(exchange='',
                          routing_key='measurement',
                          body=Z_str,
                          properties=pika.BasicProperties(
                             delivery_mode = 2, # make message persistent
                          ))
            print 'Done sending measurment'

        elif num_PC == 4:
            channel1.basic_publish(exchange='',
                          routing_key='measurement',
                          body=Z_str,
                          properties=pika.BasicProperties(
                             delivery_mode = 2, # make message persistent
                          ))
            channel1.basic_publish(exchange='',
                          routing_key='measurement',
                          body=Z_str,
                          properties=pika.BasicProperties(
                             delivery_mode = 2, # make message persistent
                          ))
            channel1.basic_publish(exchange='',
                          routing_key='measurement',
                          body=Z_str,
                          properties=pika.BasicProperties(
                             delivery_mode = 2, # make message persistent
                          ))
            channel1.basic_publish(exchange='',
                          routing_key='measurement',
                          body=Z_str,
                          properties=pika.BasicProperties(
                             delivery_mode = 2, # make message persistent
                          ))
            print 'Done sending measurment'

        elif num_PC == 8:
            channel1.basic_publish(exchange='',
                          routing_key='measurement',
                          body=Z_str,
                          properties=pika.BasicProperties(
                             delivery_mode = 2, # make message persistent
                          ))
            channel1.basic_publish(exchange='',
                          routing_key='measurement',
                          body=Z_str,
                          properties=pika.BasicProperties(
                             delivery_mode = 2, # make message persistent
                          ))
            channel1.basic_publish(exchange='',
                          routing_key='measurement',
                          body=Z_str,
                          properties=pika.BasicProperties(
                             delivery_mode = 2, # make message persistent
                          ))
            channel1.basic_publish(exchange='',
                          routing_key='measurement',
                          body=Z_str,
                          properties=pika.BasicProperties(
                             delivery_mode = 2, # make message persistent
                          ))
            channel1.basic_publish(exchange='',
                          routing_key='measurement',
                          body=Z_str,
                          properties=pika.BasicProperties(
                             delivery_mode = 2, # make message persistent
                          ))
            channel1.basic_publish(exchange='',
                          routing_key='measurement',
                          body=Z_str,
                          properties=pika.BasicProperties(
                             delivery_mode = 2, # make message persistent
                          ))
            channel1.basic_publish(exchange='',
                          routing_key='measurement',
                          body=Z_str,
                          properties=pika.BasicProperties(
                             delivery_mode = 2, # make message persistent
                          ))
            channel1.basic_publish(exchange='',
                          routing_key='measurement',
                          body=Z_str,
                          properties=pika.BasicProperties(
                             delivery_mode = 2, # make message persistent
                          ))
            print 'Done sending measurment'

        else:
            print("Skip")

        if num_PC == 2:              
            channel2.queue_declare(queue='w_and_p1', durable=True)
            print ' [*] Waiting for messages(client1).'

            channel2.basic_qos(prefetch_count=1)
            channel2.basic_consume(callback,
                                queue='w_and_p1')

            channel2.start_consuming()
            
            w_p_list = w_and_p.split('A')
            weight_list = w_p_list[0].split()
            w1 = map(float, weight_list)

            particle_list = w_p_list[1].split()
            change_float = map(float, particle_list)
            particle1 = [change_float[i:i+3] for i in range(0, len(change_float),3)]
            
            print 'Done receiving w_and_p1'

            channel2.queue_declare(queue='w_and_p2', durable=True)
            print ' [*] Waiting for messages(client2).'

            channel2.basic_qos(prefetch_count=1)
            channel2.basic_consume(callback,
                                queue='w_and_p2')

            channel2.start_consuming()
            
            w_p_list = w_and_p.split('A')
            weight_list = w_p_list[0].split()
            w2 = map(float, weight_list)

            particle_list = w_p_list[1].split()
            change_float = map(float, particle_list)
            particle2 = [change_float[i:i+3] for i in range(0, len(change_float),3)]
            
            print 'Done receiving w_and_p2'

            w = w1 + w2
            p = particle1 + particle2

        elif num_PC == 4:
            channel2.queue_declare(queue='w_and_p1', durable=True)
            print ' [*] Waiting for messages(client1).'

            channel2.basic_qos(prefetch_count=1)
            channel2.basic_consume(callback,
                                queue='w_and_p1')

            channel2.start_consuming()
            
            w_p_list = w_and_p.split('A')
            weight_list = w_p_list[0].split()
            w1 = map(float, weight_list)

            particle_list = w_p_list[1].split()
            change_float = map(float, particle_list)
            particle1 = [change_float[i:i+3] for i in range(0, len(change_float),3)]
            
            print 'Done receiving w_and_p1'

            channel2.queue_declare(queue='w_and_p2', durable=True)
            print ' [*] Waiting for messages(client2).'

            channel2.basic_qos(prefetch_count=1)
            channel2.basic_consume(callback,
                                queue='w_and_p2')

            channel2.start_consuming()
            
            w_p_list = w_and_p.split('A')
            weight_list = w_p_list[0].split()
            w2 = map(float, weight_list)

            particle_list = w_p_list[1].split()
            change_float = map(float, particle_list)
            particle2 = [change_float[i:i+3] for i in range(0, len(change_float),3)]
            
            print 'Done receiving w_and_p2'

            channel2.queue_declare(queue='w_and_p3', durable=True)
            print ' [*] Waiting for messages(client3).'

            channel2.basic_qos(prefetch_count=1)
            channel2.basic_consume(callback,
                                queue='w_and_p3')

            channel2.start_consuming()
            
            w_p_list = w_and_p.split('A')
            weight_list = w_p_list[0].split()
            w1 = map(float, weight_list)

            particle_list = w_p_list[1].split()
            change_float = map(float, particle_list)
            particle1 = [change_float[i:i+3] for i in range(0, len(change_float),3)]
            
            print 'Done receiving w_and_p3'

            channel2.queue_declare(queue='w_and_p4', durable=True)
            print ' [*] Waiting for messages(client4).'

            channel2.basic_qos(prefetch_count=1)
            channel2.basic_consume(callback,
                                queue='w_and_p4')

            channel2.start_consuming()
            
            w_p_list = w_and_p.split('A')
            weight_list = w_p_list[0].split()
            w2 = map(float, weight_list)

            particle_list = w_p_list[1].split()
            change_float = map(float, particle_list)
            particle2 = [change_float[i:i+3] for i in range(0, len(change_float),3)]
            
            print 'Done receiving w_and_p4'

            w = w1 + w2 + w3 + w4
            p = particle1 + particle2 + particle3 + particle4

        elif num_PC == 8:
            channel2.queue_declare(queue='w_and_p1', durable=True)
            print ' [*] Waiting for messages(client1).'

            channel2.basic_qos(prefetch_count=1)
            channel2.basic_consume(callback,
                                queue='w_and_p1')

            channel2.start_consuming()
            
            w_p_list = w_and_p.split('A')
            weight_list = w_p_list[0].split()
            w1 = map(float, weight_list)

            particle_list = w_p_list[1].split()
            change_float = map(float, particle_list)
            particle1 = [change_float[i:i+3] for i in range(0, len(change_float),3)]
            
            print 'Done receiving w_and_p1'

            channel2.queue_declare(queue='w_and_p2', durable=True)
            print ' [*] Waiting for messages(client2).'

            channel2.basic_qos(prefetch_count=1)
            channel2.basic_consume(callback,
                                queue='w_and_p2')

            channel2.start_consuming()
            
            w_p_list = w_and_p.split('A')
            weight_list = w_p_list[0].split()
            w2 = map(float, weight_list)

            particle_list = w_p_list[1].split()
            change_float = map(float, particle_list)
            particle2 = [change_float[i:i+3] for i in range(0, len(change_float),3)]
            
            print 'Done receiving w_and_p2'

            channel2.queue_declare(queue='w_and_p3', durable=True)
            print ' [*] Waiting for messages(client3).'

            channel2.basic_qos(prefetch_count=1)
            channel2.basic_consume(callback,
                                queue='w_and_p3')

            channel2.start_consuming()
            
            w_p_list = w_and_p.split('A')
            weight_list = w_p_list[0].split()
            w1 = map(float, weight_list)

            particle_list = w_p_list[1].split()
            change_float = map(float, particle_list)
            particle1 = [change_float[i:i+3] for i in range(0, len(change_float),3)]
            
            print 'Done receiving w_and_p3'

            channel2.queue_declare(queue='w_and_p4', durable=True)
            print ' [*] Waiting for messages(client4).'

            channel2.basic_qos(prefetch_count=1)
            channel2.basic_consume(callback,
                                queue='w_and_p4')

            channel2.start_consuming()
            
            w_p_list = w_and_p.split('A')
            weight_list = w_p_list[0].split()
            w2 = map(float, weight_list)

            particle_list = w_p_list[1].split()
            change_float = map(float, particle_list)
            particle2 = [change_float[i:i+3] for i in range(0, len(change_float),3)]
            
            print 'Done receiving w_and_p4'

            channel2.queue_declare(queue='w_and_p5', durable=True)
            print ' [*] Waiting for messages(client5).'

            channel2.basic_qos(prefetch_count=1)
            channel2.basic_consume(callback,
                                queue='w_and_p5')

            channel2.start_consuming()
            
            w_p_list = w_and_p.split('A')
            weight_list = w_p_list[0].split()
            w1 = map(float, weight_list)

            particle_list = w_p_list[1].split()
            change_float = map(float, particle_list)
            particle1 = [change_float[i:i+3] for i in range(0, len(change_float),3)]
            
            print 'Done receiving w_and_p5'

            channel2.queue_declare(queue='w_and_p6', durable=True)
            print ' [*] Waiting for messages(client6).'

            channel2.basic_qos(prefetch_count=1)
            channel2.basic_consume(callback,
                                queue='w_and_p6')

            channel2.start_consuming()
            
            w_p_list = w_and_p.split('A')
            weight_list = w_p_list[0].split()
            w2 = map(float, weight_list)

            particle_list = w_p_list[1].split()
            change_float = map(float, particle_list)
            particle2 = [change_float[i:i+3] for i in range(0, len(change_float),3)]
            
            print 'Done receiving w_and_p6'

            channel2.queue_declare(queue='w_and_p7', durable=True)
            print ' [*] Waiting for messages(client7).'

            channel2.basic_qos(prefetch_count=1)
            channel2.basic_consume(callback,
                                queue='w_and_p7')

            channel2.start_consuming()
            
            w_p_list = w_and_p.split('A')
            weight_list = w_p_list[0].split()
            w1 = map(float, weight_list)

            particle_list = w_p_list[1].split()
            change_float = map(float, particle_list)
            particle1 = [change_float[i:i+3] for i in range(0, len(change_float),3)]
            
            print 'Done receiving w_and_p7'

            channel2.queue_declare(queue='w_and_p8', durable=True)
            print ' [*] Waiting for messages(client8).'

            channel2.basic_qos(prefetch_count=1)
            channel2.basic_consume(callback,
                                queue='w_and_p8')

            channel2.start_consuming()
            
            w_p_list = w_and_p.split('A')
            weight_list = w_p_list[0].split()
            w2 = map(float, weight_list)

            particle_list = w_p_list[1].split()
            change_float = map(float, particle_list)
            particle2 = [change_float[i:i+3] for i in range(0, len(change_float),3)]
            
            print 'Done receiving w_and_p8'

            w = w1 + w2 + w3 + w4 + w5 + w6 + w7 + w8
            p = particle1 + particle2 + particle3 + particle4 + particle5 + particle6 + particle7 + particle8

        else:
            w = map(lambda particle: measurement_prob(room_plan, particle, Z), p)

        # Resampling
        print 'Resampling'
        p2 = []
        index = int(random.random() * N)
        beta = 0.0
        mw = max(w)
        for i in range(N):
            beta += random.random() * 2.0 * mw
            while beta > w[index]:
                beta -= w[index]
                index = (index + 1) % N
            p2.append(p[index])
        p = p2
       

        # save updated particles
        f = open('iteration'+str(iter_count).zfill(3)+'.dat', 'w')
        map(lambda (x,y,_): f.write(str(x)+'\t'+str(y)+'\n'), p)
        f.close()

        estimation = get_position(p)
        print "position : ", "(x =", estimation[0], ", y =", estimation[1], ", theta =", estimation[2], ")"
        print "variance : ", "(x =", estimation[3], ", y =", estimation[4], ", theta =", estimation[5], ')\n'

        #When x, y and d variances less than 0.1 every 4 position,  the robot's position is adjusted.
        if (iter_count % 4 ) == 0:
            if estimation[3] < 0.1 and estimation[4] < 0.1 and estimation[5] < 0.1:
                print "Adjust location"
                particle_x = estimation[0]
                particle_y = estimation[1]
                particle_d = estimation[2]
                original_x = motions[path_count][2][0]
                original_y = motions[path_count][2][1]
                original_d = motions[path_count][2][2]
                
                if estimation[2] < 0:
                    particle_d += 2*pi
                elif estimation[2] > 2*pi:
                    particle_d -= 2*pi
                    
                adjust_position = move_position(round(particle_x, 3), round(particle_y, 3), particle_d, original_x, original_y, original_d)
                
        iter_count += 1

        path_count += 1

    complete = 0

    return (motions, path, p, iter_count, complete, position)


#create particles
def particle_filter(motions, path, goal, N=8000):

    # Make particles

    world_x_size = abs(world_x_range[1] - world_x_range[0])
    world_y_size = abs(world_y_range[1] - world_y_range[0])
    p = []
    while len(p) < N:
        # particle is a vector (x,y,bearing)
        particle = (random.random() * world_x_size + world_x_range[0],
                    random.random() * world_y_size + world_y_range[0],
                    random.random() * 2.0 * pi)
        # add only particles inside our room
        if lineutils.point_inside_polygon(room_plan, particle):
            p.append(particle)

    position = []


    for i in range(len(motions)):
        position.append(motions[i][2])

    iter_count = 0

    complete = 1
    
    #complete : 0, not complete : 1
    while(complete == 1):
        information = prediction_and_measurement(motions, path, goal, N, p, iter_count, complete, position)
        motions = information[0]
        path = information[1]
        p = information[2]
        iter_count = information[3]
        complete = information[4]
        position = information[5]
        
    return get_position(p)


#rabbitMQ rpc communication.
class FibonacciRpcClient(object):
    def __init__(self):
        self.userinfo = pika.PlainCredentials('bbb','123123')
        self.connection = pika.BlockingConnection(pika.ConnectionParameters(
                host='172.21.11.242', credentials=self.userinfo))

        self.channel = self.connection.channel()

        result = self.channel.queue_declare(exclusive=True)
        self.callback_queue = result.method.queue

        self.channel.basic_consume(self.on_response, no_ack=True,
                                   queue=self.callback_queue)

    def on_response(self, ch, method, props, body):
        if self.corr_id == props.correlation_id:
            self.response = body

    def call(self, n):
        self.response = None
        self.corr_id = str(uuid.uuid4())
        self.channel.basic_publish(exchange='',
                                   routing_key='rpc_queue',
                                   properties=pika.BasicProperties(
                                         reply_to = self.callback_queue,
                                         correlation_id = self.corr_id,
                                         ),
                                   body=str(n))
        while self.response is None:
            self.connection.process_data_events()
        return str(self.response)


# Main application class with run() function as an entry point to the
# application
class Client():

    for i in range(len(grid)):
        print grid[i]

    coordinate = make_coordinate(grid)

    num_PC = input("The number of PCs(1, 2, 4, 8) : ")
    
    init = input("start position(x,y) : ")
    goal = input("Final position(x,y) : ")

    path = search(init,goal, None)
    
    time.sleep(1.5)

    while True:
        
        # Read room plan and create list with wall coordinates
        with open('plan.dat', 'r') as planfile:
            planreader = csv.reader(planfile, delimiter='\t')
            b = None
            for (x, y) in planreader:
                # Calculate world boundaries
                if float(x) < world_x_range[0]:
                    world_x_range[0] = float(x)
                elif float(x) > world_x_range[1]:
                    world_x_range[1] = float(x)

                if float(y) < world_y_range[0]:
                    world_y_range[0] = float(y)
                elif float(y) > world_y_range[1]:
                    world_y_range[1] = float(y)

                # Construct wall segment and add to the room_plan
                if b is not None:
                    a = b
                    b = (float(x), float(y))
                    room_plan.append((a,b))
                else:
                    b = (float(x), float(y))

            # Create proxy interface to our robot's chassis (left and
            # right wheels) using parameters (host, port, etc.) specified
            # in the configuration file as Chassis.proxy property
 

        # Make some motions and estimate resulting position using our particle filter
        motions = generate_motions('trajectory.dat', 0.0)

        estimated_position = particle_filter(motions, path, goal)
        print "Estimated final position after motion: ", estimated_position


if __name__ == "__main__":
    app = Client()
