import cv2
import numpy as np
import os
import torch 
import torch.nn as nn
import torchvision
import torchvision.transforms as transforms

REAL = True

class ConvNet(nn.Module):
    def __init__(self, num_classes=10):
        super(ConvNet, self).__init__()
        self.layer1 = nn.Sequential(
            nn.Conv2d(3, 16, kernel_size=5, stride=1, padding=2),
            nn.BatchNorm2d(16),
            nn.ReLU(),
            nn.MaxPool2d(kernel_size=2, stride=2))
        self.layer2 = nn.Sequential(
            nn.Conv2d(16, 32, kernel_size=5, stride=1, padding=2),
            nn.BatchNorm2d(32),
            nn.ReLU(),
            nn.MaxPool2d(kernel_size=2, stride=2))
        self.fc = nn.Linear(32 * 120 * 160, num_classes)
        
    def forward(self, x):
        out = self.layer1(x)
        #print('layer1', out.size())
        out = self.layer2(out)
        #print('layer2', out.size())
        out = out.reshape(out.size(0), -1)
        out = self.fc(out)
        #print('fc', out.size())
        return out

class RoboboImageInfo:

    def __init__(self, image):
        self.label = 0 # 0 prey, 1, predator
        self.x = 0.0
        self.y = 0.0
        self.image = image
        self.height = image.shape[0]
        self.width = image.shape[1]
        
device = torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')
model = ConvNet().to(device)

def get_countours(image, mask):
       
    dilation = cv2.GaussianBlur(mask, (7,7),0)

    # remove the non-target regions
    target = cv2.bitwise_and(image, image, mask=dilation)    

    # binary image
    ret, binary = cv2.threshold(dilation, 128, 255, cv2.THRESH_BINARY)   
    
    #print(cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE))
    contours, hierarchy = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)    
    
    #cv2.imshow('hsv', hsv)
    '''cv2.imshow('dilation', dilation)
    cv2.imshow('target', target)
    cv2.imshow('binary', binary)
    cv2.imshow('Mask', mask)    '''
    
    #cv2.imshow("prod", dilation)
    
    return contours
    
def draw_bounding_box(image, contours, target_name):

    rects = []
    contours = contours[::-1]

    for i in contours:  # iterate all contours        
            
        x, y, w, h = cv2.boundingRect(i)  # the coordinate of point left-top corner and length, width
        
        rects += [[x,y,w,h,target_name]]
        
        if w * h < 200:
            #print("Too small area:", w*h)
            continue
            
        #print("AREA:", w*h)
        
        # plot the bounding box
        cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255,), 3)        
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(image, str(target_name), (x - 10, y - 10), font, 1, (0, 0, 255), 2)  # +- 10 for better display

    return image, rects

def detect_robobo(image, real=False):   
    
    #output = model(torch.from_numpy(np.random.random((1, 3, 480, 640))).type(torch.FloatTensor).cuda())
    
    #return [], image
    
    img_copy = image.copy()
    
    #cv2.imshow('xxx', image)

    if not real:

        lower_predator1 = np.array([0,10,20]) 
        upper_predator1 = np.array([30,255,255])

        lower_predator2 = np.array([140,10,20])
        upper_predator2 = np.array([180,255,255])

        lower_green = np.array([30,20,20]) # 53, 110, 217 #99 132 177 # 54, 73, ?
        upper_green = np.array([70,255,255])      
        
        # change to hsv model
        hsv = cv2.cvtColor(img_copy, cv2.COLOR_BGR2HSV)

        # detect green    
        green_mask = cv2.inRange(hsv, lower_green, upper_green)
        
        # detect red
        mask1 = cv2.inRange(hsv, lower_predator1, upper_predator1)        
        mask2 = cv2.inRange(hsv, lower_predator2, upper_predator2)
        
        red_mask = mask1 + mask2        

        # search contours and rank them by the size of areas
        #print(cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE))    

        target_name = "Prey"
        contours = get_countours(image, green_mask)
        img_copy, green_rects = draw_bounding_box(img_copy, contours, target_name)
        
        target_name = "Predator"
        contours = get_countours(image, red_mask)
        img_copy, red_rects = draw_bounding_box(img_copy, contours, target_name)    
        
        rects = green_rects + red_rects
    
    else:
    
        lower_predator1 = np.array([0,150,100]) 
        upper_predator1 = np.array([5,255,255])

        lower_predator2 = np.array([175,150,100])
        upper_predator2 = np.array([180,255,255])

        lower_green = np.array([40,60,60]) # 53, 110, 217 #99 132 177 # 54, 73, ?
        upper_green = np.array([70,255,255])      
        
        # change to hsv model
        hsv = cv2.cvtColor(img_copy, cv2.COLOR_BGR2HSV)

        # detect greeny
        green_mask = cv2.inRange(hsv, lower_green, upper_green)
        
        # detect red
        mask1 = cv2.inRange(hsv, lower_predator1, upper_predator1)        
        mask2 = cv2.inRange(hsv, lower_predator2, upper_predator2)
        
        red_mask = mask1 + mask2        

        # search contours and rank them by the size of areas
        #print(cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE))    

        target_name = "Prey"
        contours = get_countours(image, green_mask)
        img_copy, green_rects = draw_bounding_box(img_copy, contours, target_name)
        
        target_name = "Predator"
        contours = get_countours(image, red_mask)
        img_copy, red_rects = draw_bounding_box(img_copy, contours, target_name)    
        
        rects = green_rects + red_rects
    
    #cv2.waitKey(0)
    #cv2.destroyAllWindows()
    
    return rects, img_copy
              
    
if __name__ == "__main__":
    for filename in os.listdir("images/"):  
        img = cv2.imread('images/' + filename)
        #img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        
        rect, img_detection = detect_robobo(img)
        #cv2.imwrite('output/' + filename, img_detection)    
        #print("save:", filename)
        
        cv2.imshow('aa', img_detection)

        cv2.waitKey(1)
        
        
    cv2.destroyAllWindows()
