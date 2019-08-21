import torch
import torch.nn as nn
import torch.nn.functional as F


class Net(nn.Module):

    def __init__(self, input_dim, output_dim):
        super(Net, self).__init__()        
        self.fc1 = nn.Linear(input_dim, 4)
        #self.fc2 = nn.Linear(20, 10)
        self.fc3 = nn.Linear(4, output_dim)

    def forward(self, x):
        x = x.to(torch.double)
        #print("input:", x)
        #print(self.fc1.weight)
        #print(self.fc1.bias)
        # Max pooling over a (2, 2) window        
        #print(self.fc1)
        x = F.relu(self.fc1(x))
        
        #x = F.relu(self.fc2(x))
        x = F.tanh(self.fc3(x))
        
        #print("output:", x)
        return x

    def num_flat_features(self, x):
        size = x.size()[1:]  # all dimensions except the batch dimension
        num_features = 1
        for s in size:
            num_features *= s
        return num_features


