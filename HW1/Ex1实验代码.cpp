#include "CImg.h"
#include <cmath>
#include <iostream>
using namespace cimg_library;
using namespace std;

#define PI 3.1415926535
struct Color {
	double red;
	double green;
	double blue;
	Color(double _r, double _g, double _b) : red(_r), green(_g), blue(_b) {}
};

struct point {
	double x;
	double y;
	point(double _x, double _y) : x(_x), y(_y) {}
};

double convertToRad(double degree) {
	return PI * degree / 180;
}

class MyImage {
public:
	MyImage();
	void processOne(CImg<unsigned char> &SrcImg);
	void processTwo(CImg<unsigned char> &SrcImg);
	void processThree(CImg<unsigned char> &SrcImg);
	void processThreeCImg(CImg<unsigned char> &SrcImg);
	void processFour(CImg<unsigned char> &SrcImg);
	void processFourCImg(CImg<unsigned char> &SrcImg);
	void processFive(CImg<unsigned char> &SrcImg);
	void processFiveCImg(CImg<unsigned char> &SrcImg);
	void processSix(CImg<unsigned char> &SrcImg, int chooseNumber);
	void drawCircle(CImg<unsigned char> &SrcImg, point center, double radius, Color color);
	void drawLine(CImg<unsigned char> &SrcImg, point from, double angle, double len, Color color);
};

MyImage::MyImage() {}


//步骤一：读入1.bmp文件，并用CImg.display()显示
void MyImage::processOne(CImg<unsigned char> &SrcImg) {
	SrcImg.display();
}


//步骤二：把1.bmp文件的白色区域编程红色，黑色区域变成绿色
void MyImage::processTwo(CImg<unsigned char> &SrcImg) {
	cimg_forXY(SrcImg, x, y) {
		if (SrcImg(x, y, 0) == 0 && SrcImg(x, y, 1) == 0 && SrcImg(x, y, 2) == 0) {
			SrcImg(x, y, 0) = 0;
			SrcImg(x, y, 1) = 255;
			SrcImg(x, y, 2) = 0; 
		}

		if (SrcImg(x, y, 0) == 255 && SrcImg(x, y, 1) == 255 && SrcImg(x, y, 2) == 255) {
			SrcImg(x, y, 0) = 255;
			SrcImg(x, y, 1) = 0;
			SrcImg(x, y, 2) = 0;
		}
	}
	SrcImg.display();
}


//步骤三：在图上绘制一个圆形区域，圆心坐标(50,50)，半径为30，填充颜色为蓝色
//未使用了CImg库绘图方法的步骤三
void MyImage::processThree(CImg<unsigned char> &SrcImg) {
	point center(50, 50);
	double radius = 30;
	Color color(0, 0, 255);
	drawCircle(SrcImg, center, radius, color);
}


//使用了CImg库绘图方法的步骤三
void MyImage::processThreeCImg(CImg<unsigned char> &SrcImg) {
	point center(50, 50);
	double radius = 30;
	unsigned char color[] = {0, 0, 255};
	SrcImg.draw_circle(center.x, center.y, radius, color);
}


//步骤四：在图上绘制一个图形区域，圆心坐标(50,50)，半径为3，填充颜色为黄色
//未使用了CImg库绘图方法的步骤四
void MyImage::processFour(CImg<unsigned char> &SrcImg) {
	point center(50, 50);
	double radius = 3;
	Color color(255, 255, 0);
	drawCircle(SrcImg, center, radius, color);	
}


//使用了CImg库绘图方法的步骤四
void MyImage::processFourCImg(CImg<unsigned char> &SrcImg) {
	point center(50, 50);
	double radius = 3;
	unsigned char color[] = {255, 255, 0};
	SrcImg.draw_circle(center.x, center.y, radius, color);
}

//步骤五：在图上绘制一条 在图上绘制一条 长为100的直线段，起点坐标为起点坐标为(0, 0)，方向角为35度，直线的颜色为蓝色
//未使用了CImg库绘图方法的步骤五
void MyImage::processFive(CImg<unsigned char> &SrcImg) {
	point from(0, 0);
	double angle = 35;
	double len = 100;
	Color color(0, 0, 255);
	drawLine(SrcImg, from, angle, len, color);
}

//使用了CImg库绘图方法的步骤五
void MyImage::processFiveCImg(CImg<unsigned char> &SrcImg) {
	point from(0, 0);
	double angle = 35;
	double rad = convertToRad(angle);
	unsigned char color[] = {0, 0, 255};
	double to_x = 100 * cos(rad);
	double to_y = 100 * sin(rad);
	point to(to_x, to_y);
	SrcImg.draw_line(from.x, from.y, to.x, to.y, color);
}

//根据是否选择使用了CImg库绘图方法的函数，保存为相应的bmp图片
//如果选择的是使用了CImg库绘图方法，即输入的是0，结果保存为2.bmp，否则保存为3.bmp
void MyImage::processSix(CImg<unsigned char> &SrcImg, int chooseNumber) {
	//With the use of CImg.
	if (chooseNumber == 0) {
		SrcImg.save("2.bmp");
	} 
	//Not use CImg.
	else if (chooseNumber == 1){
		SrcImg.save("3.bmp");
	}
}

//步骤三、四画圆方法的辅助函数
void MyImage::drawCircle(CImg<unsigned char> &SrcImg, point center, double radius, Color color) {
	cimg_forXY(SrcImg, x, y) {
		double x1 = pow(x - center.x, 2);
		double y1 = pow(y - center.y, 2);
		double distance = sqrt(x1 + y1);

		if (distance <= radius) {
			SrcImg(x, y, 0) = color.red;
			SrcImg(x, y, 1) = color.green;
			SrcImg(x, y, 2) = color.blue;
		}
	}
}

//步骤五画直线方法的辅助函数
void MyImage::drawLine(CImg<unsigned char> &SrcImg, point from, double angle, double len, Color color) {
	double rad = convertToRad(angle);
	double k = tan(rad);
	cimg_forXY(SrcImg, x, y) {
		int dis = from.y + k * (x - from.x);
		if (y == dis && x >= 0 && x <= len * cos(rad)) {
			SrcImg(x, y, 0) = color.red;
			SrcImg(x, y, 1) = color.green;
			SrcImg(x, y, 2) = color.blue;				
		}
	}
}

int main() {
	MyImage image;
	CImg<unsigned char> SrcImg;
	int chooseNumber = 0;
	SrcImg.load_bmp("1.bmp");
	cout << "If use CImg, then input 0, otherwise input 1" << endl;
	cout << "Please input the method number: ";
	cin >> chooseNumber;
	//若输入的结果为0，则输出的图片是使用了CImg库绘图方法的结果
	//如果需要单独测试某一个方法，请将相应的方法注释掉
	if (chooseNumber == 0) {
		image.processOne(SrcImg);
		image.processTwo(SrcImg);
		image.processThreeCImg(SrcImg);
		image.processFourCImg(SrcImg);
		image.processFiveCImg(SrcImg);
		image.processSix(SrcImg, chooseNumber);		
	}

	//若输入的结果为1，则输出的图片是未使用CImg库绘图方法的结果
	//如果需要单独测试某一个方法，请将相应的方法注释掉
	else if (chooseNumber == 1){
		image.processOne(SrcImg);
		image.processTwo(SrcImg);
		image.processThree(SrcImg);
		image.processFour(SrcImg);
		image.processFive(SrcImg);
		image.processSix(SrcImg, chooseNumber);		
	}
	return 0;
}

