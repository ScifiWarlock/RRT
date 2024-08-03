"""

Path planning with Rapidly-Exploring Random Trees (RRT)

author: Aakash(@nimrobotics)
Modified By: Omkar Tasgaonkar


"""

import cv2
import numpy as np
import math
import random
import argparse
import os
import imageio
import urllib.request

## Read the gif from the web, save to the disk
fname = "complexGif.gif"

gif = imageio.mimread(fname)
nums = len(gif)
print("Total {} frames in the gif!".format(nums))

# convert form RGB to BGR 
imgsOut = [cv2.cvtColor(img, cv2.COLOR_RGB2BGR) for img in gif]
imgs = [cv2.cvtColor(img, cv2.COLOR_RGBA2GRAY,0) for img in gif]

class Nodes:
    """Class to store the RRT graph"""
    def __init__(self, x,y):
        self.x = x
        self.y = y
        self.parent_x = []
        self.parent_y = []

# check collision
def collision(x1,y1,x2,y2):

    color=[]
    x = list(np.arange(x1,x2,(x2-x1)/100))
    y = list(((y2-y1)/(x2-x1))*(x-x1) + y1)
    #print("collision",x,y)
    if cv2.waitKey(100)&0xFF == 27:
        k = (k+1)%nums
    for i in range(len(x)):
        #print(int(x[i]),int(y[i]))
        color.append(img[int(y[i]),int(x[i])])
    if (0 in np.array(color)):
        return True #collision
    else:
        return False #no-collision

# check the  collision with obstacle and trim
def check_collision(x1,y1,x2,y2):
    _,theta = dist_and_angle(x2,y2,x1,y1)
    x=x2 + stepSize*np.cos(theta)
    y=y2 + stepSize*np.sin(theta)
    print(x2,y2,x1,y1)
    print("theta",theta)
    print("check_collision",x,y)

    # TODO: trim the branch if its going out of image area
    # print("Image shape",img.shape)
    hy,hx=img.shape[:2]
    if y<0 or y>hy or x<0 or x>hx:
        print("Point out of image bound")
        directCon = False
        nodeCon = False
    else:
        # check direct connection
        if collision(x,y,end[0],end[1]):
            directCon = False
        else:
            directCon=True

        # check connection between two nodes
        if collision(x,y,x2,y2):
            nodeCon = False
        else:
            nodeCon = True

    return(x,y,directCon,nodeCon)

# return dist and angle b/w new point and nearest node
def dist_and_angle(x1,y1,x2,y2):
    dist = math.sqrt( ((x1-x2)**2)+((y1-y2)**2) )
    angle = math.atan2(y2-y1, x2-x1)
    return(dist,angle)

# return the neaerst node index
def nearest_node(x,y):
    temp_dist=[]
    for i in range(len(node_list)):
        dist,_ = dist_and_angle(x,y,node_list[i].x,node_list[i].y)
        temp_dist.append(dist)
    return temp_dist.index(min(temp_dist))

# generate a random point in the image space
def rnd_point(h,l):
    new_y = random.randint(0, h)
    new_x = random.randint(0, l)
    return (new_x,new_y)

def pathDraw():
    for j in range(len(node_list)):
        cv2.circle(img2, (int(node_list[j].x),int(node_list[j].y)), 2,(0,0,255),thickness=3, lineType=8)
       
        if len(node_list[j].parent_x) != 0 and len(node_list[j].parent_y) != 0:
            for v in range(len(node_list[j].parent_x) -1):
                cv2.line(img2, (int(node_list[j].parent_x[v]),int(node_list[j].parent_y[v])), 
                         (int(node_list[j].parent_x[v+1]),int(node_list[j].parent_y[v+1])), (0,255,0), thickness=1, lineType=8)
            
    cv2.waitKey(100)


def RRT(start, end, stepSize):
   
    k = 0
    global img, img2

    img = imgs[k].copy()
    img2 = imgsOut[k].copy()

    cv2.imshow("gif", img2)
    h,l= img.shape[:2] # dim of the loaded image
    print(img.shape) # (384, 683)
    # print(h,l)

    # insert the starting point in the node class
    # node_list = [0] # list to store all the node points         
    node_list[0] = Nodes(start[0],start[1])
    node_list[0].parent_x.append(start[0])
    node_list[0].parent_y.append(start[1])

    # display start and end
    cv2.circle(img2, (start[0],start[1]), 5,(0,0,255),thickness=3, lineType=8)
    cv2.circle(img2, (end[0],end[1]), 5,(0,0,255),thickness=3, lineType=8)
    
    i=1
    pathFound = False
    while pathFound==False:
        nx,ny = rnd_point(h,l)
        print("Random points:",nx,ny)

        nearest_ind = nearest_node(nx,ny)
        nearest_x = node_list[nearest_ind].x
        nearest_y = node_list[nearest_ind].y
        print("Nearest node coordinates:",nearest_x,nearest_y)

        #check direct connection
        tx,ty,directCon,nodeCon = check_collision(nx,ny,nearest_x,nearest_y)
        print("Check collision:",tx,ty,directCon,nodeCon)

        if directCon and nodeCon:
            print("Node can connect directly with end")
            node_list.append(i)
            node_list[i] = Nodes(tx,ty)
            node_list[i].parent_x = node_list[nearest_ind].parent_x.copy()
            node_list[i].parent_y = node_list[nearest_ind].parent_y.copy()
            node_list[i].parent_x.append(tx)
            node_list[i].parent_y.append(ty)

            cv2.circle(img2, (int(tx),int(ty)), 2,(0,0,255),thickness=3, lineType=8)
            cv2.line(img2, (int(tx),int(ty)), (int(node_list[nearest_ind].x),int(node_list[nearest_ind].y)), (0,255,0), thickness=1, lineType=8)
            #cv2.line(img2, (int(tx),int(ty)), (end[0],end[1]), (255,0,0), thickness=2, lineType=8)
            cv2.line(img2, (int(tx),int(ty)), (end[0],end[1]), (0,255,0), thickness=1, lineType=8)

            print("Path has been found")
            #print("parent_x",node_list[i].parent_x)
            for j in range(len(node_list[i].parent_x)-1):
                cv2.line(img2, (int(node_list[i].parent_x[j]),int(node_list[i].parent_y[j])), (int(node_list[i].parent_x[j+1]),int(node_list[i].parent_y[j+1])), (255,0,0), thickness=2, lineType=8)

            pathDraw()
            cv2.imwrite("media/"+str(i)+".jpg",img2)
            cv2.imwrite("final_out.jpg",img2)

            break


        elif nodeCon:
            print("Nodes connected")
            node_list.append(i)
            node_list[i] = Nodes(tx,ty)
            node_list[i].parent_x = node_list[nearest_ind].parent_x.copy()
            node_list[i].parent_y = node_list[nearest_ind].parent_y.copy()
            # print(i)
            # print(node_list[nearest_ind].parent_y)
            node_list[i].parent_x.append(tx)
            node_list[i].parent_y.append(ty)
            i=i+1
            # display
            cv2.circle(img2, (int(tx),int(ty)), 2,(0,0,255),thickness=3, lineType=8)
            cv2.line(img2, (int(tx),int(ty)), (int(node_list[nearest_ind].x),int(node_list[nearest_ind].y)), (0,255,0), thickness=1, lineType=8)
            cv2.imwrite("media/"+str(i)+".jpg",img2)
            cv2.imshow("gif",img2)
            cv2.waitKey(100)
           
            if i % 1 == 0:
                k =  (k + 1)%6
                img = imgs[k].copy()
                img2 = imgsOut[k].copy()
                cv2.imshow("gif", img2)
           
            cv2.circle(img2, (start[0],start[1]), 5,(0,0,255),thickness=3, lineType=8)
            cv2.circle(img2, (end[0],end[1]), 5,(0,0,255),thickness=3, lineType=8)
            pathDraw()
            continue

        else:
            print("No direct con. and no node con. :( Generating new rnd numbers")
            continue

    

def draw_circle(event,x,y,flags,param):
    global coordinates
    if event == cv2.EVENT_LBUTTONDBLCLK:
        cv2.circle(img,(x,y),5,(255,0,0),-1)
        coordinates.append(x)
        coordinates.append(y)



if __name__ == '__main__':

    parser = argparse.ArgumentParser(description = 'Below are the params:')

    parser.add_argument('-start', type=int, default=[10,10], metavar='startCoord', dest='start', nargs='+',
                    help='Starting position in the maze')
    parser.add_argument('-stop', type=int, default=[900,400], metavar='stopCoord', dest='stop', nargs='+',
                    help='End position in the maze')
    parser.add_argument('-s', type=int, default=10,metavar='Stepsize', action='store', dest='stepSize',
                    help='Step-size to be used for RRT branches')

    args = parser.parse_args()

    # remove previously stored data
    try:
       os.system("rm -rf media")
       os.mkdir("media")
    except:
       print("Dir already clean")


    ## Display the gif
    # Get the Input params
    start = tuple(args.start) #(20,20) # starting coordinate
    end = tuple(args.stop) #(450,250) # target coordinate
    stepSize = args.stepSize # stepsize for RRT    

    node_list = [0] # list to store all the node points

    # run the RRT algorithm 
    RRT(start, end, stepSize)

