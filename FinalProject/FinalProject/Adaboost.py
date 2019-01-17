import pandas as pd
import numpy as np
import subprocess
import scipy.io as sio
import os
import struct
import xlwt
import xlrd
from xlutils.copy import copy

from skimage import transform
from skimage import io
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
    #arr = translate(filename)
    return adaboost.predict(arr)


# Test the digit in the A4 paper.
def readTestImage(path):
    # 1. Convert the image into grey image.
    image = Image.open(path).convert('L')
    width, height = image.size
    pixel = image.load()
    # image.show()
    
    """
    for i in range(width):
        for j in range(height):
            if pixel[i, j] >= 100:
                pixel[i, j] = 0
            else:
                pixel[i, j] = 255 - pixel[i, j]
    """

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
                             n_estimators=800, \
                             learning_rate=0.05, \
                             algorithm='SAMME.R')

    
    # Train model or load the model
    if (os.path.isfile("./mnist/adaboost.m")):
        adaboost = joblib.load("./mnist/adaboost.m")
    else:
        train_model(adaboost, "./mnist/adaboost.m")

    book = xlwt.Workbook(encoding='utf-8', style_compression=0)
    sheet = book.add_sheet('test', cell_overwrite_ok=False)
    tb_head = {
        u'学号',
        u'身份证',
        u'电话号码'
    }
    
    path = "./Segmentation"
    fileNum = 0
    for lists in os.listdir(path):
        sub_path = os.path.join(path, lists)
        if os.path.isfile(sub_path):
            fileNum = fileNum+1

    arrDigit = []

    # Test the digit from A4 paper.
    for i in range(0, fileNum):
        digit = digitPredictFromPaper(adaboost, "./Segmentation/" + str(i) + ".bmp")
        arrDigit.append(digit)
        print("The digit of this paper is: {0}".format(digit))

    #sheet.write(0, 0, arrDigit)

    intersection_data = open('./Intersection.txt')
    for i in range(0, 4):
        strs = intersection_data.readline()
        strd = "顶点" + str(i+1)
        sheet.write(i, 0, strd)
        sheet.write(i, 1, strs)
    intersection_data.close()

    arrList = []
    arrNum = open("./Row.txt")
    line = arrNum.readline()
    while line:
        # print(line)
        num = int(line)
        arrList.append(num)
        line = arrNum.readline()

    arrNum.close()
    for i in range(0, len(arrList)):
        print(arrList[i])


    arrLength = len(arrList)
    pos = 5
    k = 0
    u = 0

    while k < arrLength:
        str1 = ""
        for i in range(0, arrList[k]):
            if k < arrList[k]:
                if u >= len(arrDigit):
                    continue
                str1 = str1 + str(arrDigit[u])[1]
                u = u+1
        k = k+1
        sheet.write(pos, 0, "学号")
        sheet.write(pos, 1, str1)
        pos = pos+1

        if k >= arrLength:
            break

        str2 = ""
        for i in range(0, arrList[k]):
            if k < arrList[k]:
                if u >= len(arrDigit):
                    continue
                str2 = str2 + str(arrDigit[u])[1]
                u = u+1
        k = k+1
        sheet.write(pos, 0, "手机号")
        sheet.write(pos, 1, str2)
        pos = pos+1

        if k >= arrLength:
            break

        str3 = ""
        for i in range(0, arrList[k]):
            if k < arrList[k]:
                if u >= len(arrDigit):
                    continue
                str3 = str3 + str(arrDigit[u])[1]
                u = u+1
        k = k+1
        sheet.write(pos, 0, "身份证号")
        sheet.write(pos, 1, str3)
        pos = pos+2
        if k >= arrLength:
            break



    book.save(r'./test.xls')

