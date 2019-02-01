import os

DATA_PATH = "data/"

os.mkdir(DATA_PATH + "fitness")
os.mkdir(DATA_PATH + "weights")
os.mkdir(DATA_PATH + "distance")
os.mkdir(DATA_PATH + "tracking")
os.mkdir(DATA_PATH + "es")

os.mkdir(DATA_PATH + "distance/cma")
os.mkdir(DATA_PATH + "distance/es")
os.mkdir(DATA_PATH + "distance/bo")
os.mkdir(DATA_PATH + "distance/pk")

os.mkdir(DATA_PATH + "weights/cma")
os.mkdir(DATA_PATH + "weights/es")
os.mkdir(DATA_PATH + "weights/bo")
