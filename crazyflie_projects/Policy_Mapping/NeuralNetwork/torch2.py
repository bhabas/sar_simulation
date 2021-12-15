import numpy as np
from sklearn.datasets import make_moons
from sklearn.model_selection import train_test_split
import matplotlib.pyplot as plt
import torch
import torch.nn as nn
# %matplotlib inline

print("Using PyTorch Version %s" %torch.__version__)