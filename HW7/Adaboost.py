import pandas as pd
import numpy as np
import subprocess
import scipy.io as sio
import os
import struct

from sklearn.model_selection import train_test_split
from sklearn.metrics import accuracy_score
from sklearn.ensemble import AdaBoostClassifier
from sklearn.neural_network import MLPClassifier
from sklearn.externals import joblib
from sklearn.tree import DecisionTreeClassifier
from PIL import Image

# Load mnist file method refers to https://www.cnblogs.com/xianhan/p/9145966.html
def load_mnist(path, kind='train'):
    labels_path = os.path.join(path, '%s-labels.idx1-ubyte' % kind)
    images_path = os.path.join(path, '%s-images.idx3-ubyte' % kind)
    with open(labels_path, 'rb') as lbpath:
        magic, n = struct.unpack('>II',
                          lbpath.read(8))
        labels = np.fromfile(lbpath, dtype=np.uint8)

    with open(images_path, 'rb') as imgpath:
        magic, num, rows, cols = struct.unpack('>IIII',
                                        imgpath.read(16))
        images = np.fromfile(imgpath, dtype=np.uint8).reshape(len(labels), 784)

    return images, labels

def train_model(adaboost, dir):
    # Load the train data and labels.
    [train_images, train_labels] = load_mnist('./mnist')
    train_labels = train_labels.astype('int32')
    train_images = train_images.astype('float')
    train_images /= 255.0
    # print(train_images)
    # print(train_labels)

    # Train the adaboost model.
    print("Train Adaboost Model")
    adaboost.fit(train_images, train_labels)

    # Save the model.
    joblib.dump(adaboost, dir)
    adaboost = joblib.load(dir)

    print("Train Adaboost Model Complete!")

    # Load the test data and labels.
    [test_images, test_labels] = load_mnist('./mnist', kind = 't10k')
    test_labels = test_labels.astype('int32')
    test_images = test_images.astype('float')
    test_images /= 255.0
    # print(test_labels)

    # Get the predict digit.
    predict_digit = list(adaboost.predict(test_images))
    # print(predict_digit)
    # Get the accuracy rate.
    print("The accuracy score is: {:.4f}".format(accuracy_score(test_labels, predict_digit)))



def digitPredictFromPaper(adaboost, filename):
    arr = readTestImage(filename)
    return adaboost.predict(arr)


# Test the digit in the A4 paper.
def readTestImage(path):
    # 1. Convert the image into grey image.
    image = Image.open(path).convert('L')
    width, height = image.size
    pixel = image.load()
    # image.show()
    
    for i in range(width):
        for j in range(height):
            if pixel[i, j] >= 100:
                pixel[i, j] = 0
            else:
                pixel[i, j] = 255 - pixel[i, j]

    # 2. Resize the image into 28 * 28.
    if image.size[0] != 28 or image.size[1] != 28:
        image = image.resize((28, 28))

    # 3. Normalize the image.
    arr = []
    for i in range(28):
        for j in range(28):
            arr.append(float(image.getpixel((j, i))) / 255.0)

    # 4. Turn into a 784 dim vector.
    digitVector = np.array(arr).reshape((1, 784))
    # print(digitVector)

    # Show the image after resize.
    # image.show()

    return digitVector



if __name__ == '__main__':
    [train_images, train_labels] = load_mnist('./mnist')
    # print(train_images)
    # print(train_labels)
    
    # DecisionTreeClassifier
    adaboost = AdaBoostClassifier(DecisionTreeClassifier(max_depth=5, min_samples_split=5, min_samples_leaf=5), \
                             n_estimators=400, \
                             learning_rate=0.05, \
                             algorithm='SAMME.R')
    
    # Train model or load the model
    if (os.path.isfile("./mnist/adaboost.m")):
        adaboost = joblib.load("./mnist/adaboost.m")
    else:
        train_model(adaboost, "./mnist/adaboost.m")
    
    # Test the digit from A4 paper.
    digit = digitPredictFromPaper(adaboost, "./5.bmp")
    print("The digit of this paper is: {0}".format(digit))