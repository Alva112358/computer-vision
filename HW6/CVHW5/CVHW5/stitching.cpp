#include "stdafx.h"
#include "stitching.h"


// Constructor, initialize all the parameters and stitching.
Panorama::Panorama(string filename, int _fileNum) {
    printf("Start reading the images into the CImgList...\n");
    fileNum = _fileNum;
    for (int i = 0; i < fileNum; i++) {
        string File =  filename + to_string(i) + ".bmp";
		//cout << File << endl;
        image.push_back(CImg<float>(File.c_str()));
    }
	width = image[0]._width;
	height = image[0]._height;
	printf("Finish reading the images into the CImgList...\n");

    // Display list.
    image.display("Image", false);

    // Create cylinder image.
    createCylinderImage();

    // Create grey image.
    createGreyImage();

    // Extract SIFT features and Match features.
    createExtractFeature();

    final_result = stitching();
    final_result.display("Final", false);
    final_result.save("Panorama.bmp");
}

Panorama::~Panorama() {
    printf("Thank you for using this program!\n");
}

// 生成灰度图集合
void Panorama::createGreyImage() {
    for (int i = 0; i < fileNum; i++) {
        CImg<float> grey(width, height);
        cimg_forXY(cylinder_image[i], x, y) {
            grey(x, y) = 0.299 * cylinder_image[i](x, y, 0, 0) +
                         0.587 * cylinder_image[i](x, y, 0, 1) +
                         0.114 * cylinder_image[i](x, y, 0, 2);
        }
        grey_image.push_back(grey);
    }
}

// Create cylinder image.
void Panorama::createCylinderImage() {
    int width = image[0]._width;
    int height = image[0]._height;
    int depth = image[0]._depth;

    // Create cylinder projection for each image.
    for (int i = 0; i < fileNum; i++) {
        CImg<float> cylinder(width, height, depth, 3);
        cylinder.fill(0.0f);
        float centerX = width / 2;
        float centerY = height / 2;
        float f = width / (2 * tan(M_PI / 8));
        cimg_forXY(cylinder, x, y) {
            float theta = asin((x - centerX) / f);
            int pointX = (f * tan((x - centerX) / f) + centerX);
            int pointY = ((y - centerY) / cos(theta) + centerY);
            for (int k = 0; k < depth; k++) {
                if (pointX >= 0 && pointX < width && pointY >= 0 && pointY < height) {
                    cylinder(x, y, k, 0) = image[i](pointX, pointY, k, 0);
                    cylinder(x, y, k, 1) = image[i](pointX, pointY, k, 1);
                    cylinder(x, y, k, 2) = image[i](pointX, pointY, k, 2);
                }
                else {
                    cylinder(x, y, k, 0) = 0;
                    cylinder(x, y, k, 1) = 0;
                    cylinder(x, y, k, 2) = 0;
                }
            }
        }
        cylinder_image.push_back(cylinder);
    }

    cylinder_image.display("Cylinder", false);
}

// SIFT 特征提取部分
void Panorama::createExtractFeature() {
    for (int i = 0; i < fileNum; i++) {
        CImg<float> sourceImage(grey_image[i]);

        // 创建vl_sift_pix(float)类型的数组imageData，并将原图像中的数据复制到其中
        vl_sift_pix* imageData = new vl_sift_pix[sourceImage._width * sourceImage._height];
        for (int i = 0; i < sourceImage._width; i++) {
            for (int j = 0; j < sourceImage._height; j++) {
                imageData[j * sourceImage._width + i] = sourceImage(i, j, 0);
            }
        }

        // 创建 SIFT 过滤器，包含 SIFT 检测器和描述符
        int noctaves = 4;  // 组数
        int nlevels = 2;   // 每组的层数
        int o_min = 0;     // 第一组的索引号
        VlSiftFilt* siftFilter = NULL;
        siftFilter = vl_sift_new(sourceImage._width, sourceImage._height, noctaves, nlevels, o_min);
    
        // image[i] 的 SIFT 特征点集
        map<vector<float>, VlSiftKeypoint> feature;

        // 计算 DOG 尺度空间的第一层
        if (vl_sift_process_first_octave(siftFilter, imageData) != VL_ERR_EOF) {
            while (true) {
                // 通过 SIFT 过滤器遍历获得关键点
                vl_sift_detect(siftFilter);
                VlSiftKeypoint* pKeyPoint = siftFilter->keys;

                // 对每一个关键点进行处理
                for (int i = 0; i < siftFilter->nkeys; i++) {
                    VlSiftKeypoint tempKp = *pKeyPoint;

                    // 获取关键点的主方向和辅方向，最多四个方向
                    double angles[4];
                    int angleCount = vl_sift_calc_keypoint_orientations(siftFilter, angles, &tempKp);
                    
                    for (int j = 0; j < angleCount; j++) {
                        double tempAngle = angles[j];
                        // 计算每个方向的描述子
                        vl_sift_pix descriptors[128];
                        vl_sift_calc_keypoint_descriptor(siftFilter, descriptors, &tempKp, tempAngle);

                        vector<float> des;
                        int k = 0;
                        while (k < 128) {
                            des.push_back(descriptors[k]);
                            k++;
                        }

                        tempKp.ix = tempKp.x;
                        tempKp.iy = tempKp.y;

                        feature.insert(make_pair(des, tempKp));
                    }
                    pKeyPoint++;
                }
                if (vl_sift_process_next_octave(siftFilter) == VL_ERR_EOF) {
                    break;
                }
            }
        }
        vl_sift_delete(siftFilter);
        delete[] imageData;
        imageData = NULL;
        features.push_back(feature);
    }
}

// get random number between min and max.
int Panorama::random(int min, int max) {
    return rand() % (max-min+1) + min;
}

// SIFT 特征值提取
vector<Point> Panorama::getKeyPointMatch(map<vector<float>, VlSiftKeypoint>& feature_a, map<vector<float>, VlSiftKeypoint>& feature_b) {
    VlKDForest* forest = vl_kdforest_new(VL_TYPE_FLOAT, 128, 1, VlDistanceL1);

    float* data = new float[128 * feature_a.size()];
    int k = 0;
    for (auto it = feature_a.begin(); it != feature_a.end(); it++) {
        const vector<float>& descriptors = it->first;

        for (int i = 0; i < 128; i++) {
            data[i + 128 * k] = descriptors[i];
        }
        k++;
    }

    vl_kdforest_build(forest, feature_a.size(), data);

    vector<Point> result;

    VlKDForestSearcher* searcher = vl_kdforest_new_searcher(forest);
    VlKDForestNeighbor neighbours[2];

    for (auto it = feature_b.begin(); it != feature_b.end(); it++) {
        float *temp_data = new float[128];

        for (int i = 0; i < 128; i++) {
            temp_data[i] = (it->first)[i];
        }

        int nvisited = vl_kdforestsearcher_query(searcher, neighbours, 2, temp_data);

        float ratio = neighbours[0].distance / neighbours[1].distance;
        if (ratio < 0.5) {
            vector<float> des(128);
            for (int j = 0; j < 128; j++) {
                des[j] = data[j + neighbours[0].index * 128];
            }

        VlSiftKeypoint left = feature_a.find(des)->second;
        VlSiftKeypoint right = it->second;
        result.push_back(Point(left, right));
        }

        delete[] temp_data;
        temp_data = NULL;
    }


    vl_kdforestsearcher_delete(searcher);
    vl_kdforest_delete(forest);

    delete[] data;
    data = NULL;

    return result;
}


// 通过随机选取的四个点，生成单应性变换矩阵H，记为模型M.
// H为3*3的矩阵，因此需要4对对应点才能解出方程, 在下面写成H = [h11, h12, h13, h21, h22, h23, h31, h32, h33]^T.
MatrixXf Panorama::createHomography(vector<Point>& random_pairs) {
    if (random_pairs.size() != 4) {
        printf("Error! The number of point pairs is less than 4\n");
        exit(1);
    }

    MatrixXf Homography(8, 1);

    // 8个方程组 8个未知数
    MatrixXf UV(8, 1), A(8, 8);
    UV << random_pairs[0].b.x, random_pairs[0].b.y,
          random_pairs[1].b.x, random_pairs[1].b.y,
          random_pairs[2].b.x, random_pairs[2].b.y,
          random_pairs[3].b.x, random_pairs[3].b.y;

    A  << random_pairs[0].a.x, random_pairs[0].a.y, 1, 0, 0, 0, -random_pairs[0].a.x * random_pairs[0].b.x, -random_pairs[0].a.y * random_pairs[0].b.x,
          0, 0, 0, random_pairs[0].a.x, random_pairs[0].a.y, 1, -random_pairs[0].a.x * random_pairs[0].b.y, -random_pairs[0].a.y * random_pairs[0].b.y,
          random_pairs[1].a.x, random_pairs[1].a.y, 1, 0, 0, 0, -random_pairs[1].a.x * random_pairs[1].b.x, -random_pairs[1].a.y * random_pairs[1].b.x,
          0, 0, 0, random_pairs[1].a.x, random_pairs[1].a.y, 1, -random_pairs[1].a.x * random_pairs[1].b.y, -random_pairs[1].a.y * random_pairs[1].b.y,
          random_pairs[2].a.x, random_pairs[2].a.y, 1, 0, 0, 0, -random_pairs[2].a.x * random_pairs[2].b.x, -random_pairs[2].a.y * random_pairs[2].b.x,
          0, 0, 0, random_pairs[2].a.x, random_pairs[2].a.y, 1, -random_pairs[2].a.x * random_pairs[2].b.y, -random_pairs[2].a.y * random_pairs[2].b.y,
          random_pairs[3].a.x, random_pairs[3].a.y, 1, 0, 0, 0, -random_pairs[3].a.x * random_pairs[3].b.x, -random_pairs[3].a.y * random_pairs[3].b.x,
          0, 0, 0, random_pairs[3].a.x, random_pairs[3].a.y, 1, -random_pairs[3].a.x * random_pairs[3].b.y, -random_pairs[3].a.y * random_pairs[3].b.y;

    A = A.inverse();
    Homography = A * UV;
    return Homography;
}


// 根据给定的数据集，创建邻域点集
// pairs 表示数据集, H表示单应性变换矩阵，indices表示
vector<int> Panorama::createInlinerDataSet(vector<Point>& data_pair, MatrixXf& H, set<int>& indices) {
    // 邻域点的结果
    vector<int> inliners;

    for (int i = 0; i < data_pair.size(); i++) {
        // 除去选中的四个点，如果在indices中找不到，则可以认为是其他点
        if (indices.find(i) != indices.end()) continue;

        float real_x = data_pair[i].b.x;
        float real_y = data_pair[i].b.y;

        // 获取变换后得到的点
        if (data_pair[i].b.x == 0 || data_pair[i].b.y == 0) continue;
        int point_x = (H(0, 0) * data_pair[i].a.x + H(1, 0) * data_pair[i].a.y + H(2, 0)) / (H(6, 0) * data_pair[i].a.x + H(7, 0) * data_pair[i].a.y + 1);
        int point_y = (H(3, 0) * data_pair[i].a.x + H(4, 0) * data_pair[i].a.y + H(5, 0)) / (H(6, 0) * data_pair[i].a.x + H(7, 0) * data_pair[i].a.y + 1);

        // 利用欧式距离来判断是否是邻域点
        float dist = sqrt((point_x - real_x) * (point_x - real_x) + (point_y - real_y) * (point_y - real_y));

        // 如果该点小于设置的阈值，则认为是邻域点
        if (dist < RANSAC_THRESHOLD) {
            inliners.push_back(i);
        }
    }
    return inliners;
}


// 最小二乘解，对生成的最优点集进行一次拟合.
MatrixXf Panorama::leastSquareSolution(vector<Point>& pairs, vector<int>& inliner) {
    MatrixXf leastSquare(8, 1);
    CImg<float> A(4, inliner.size(), 1, 1, 0);
    CImg<float> b(1, inliner.size(), 1, 1, 0);

    for (int i = 0; i < inliner.size(); i++) {
        A(0, i) = pairs[inliner[i]].a.x;
        A(1, i) = pairs[inliner[i]].a.y;
        A(2, i) = pairs[inliner[i]].a.x * pairs[inliner[i]].a.y;
        A(3, i) = 1;

        b(0, i) = pairs[inliner[i]].b.x;
    }
    CImg<float> x1 = b.get_solve(A);

    for (int i = 0; i < inliner.size(); i++) {
        b(0, i) = pairs[inliner[i]].b.y;
    }
    CImg<float> x2 = b.get_solve(A);

    
    leastSquare << x1(0, 0), x1(0, 1), x1(0, 2), x1(0, 3), 
                   x2(0, 0), x2(0, 1), x2(0, 2), x2(0, 3);

    return leastSquare;
}

// RANSAC 算法.
vector<int> Panorama::RANSAC(vector<Point>& pairs) {
    srand(time(NULL));

    float p = 0.99;
    float w = 0.5;
    float n = NUM_OF_PAIR;
    int circle_number = ceil(log(1-p) / log(1-pow(w,n)));

    vector<int> best_inliner;

    while (circle_number--) {
        vector<Point> random_point_pairs;
        set<int> selected_indices;

        // 随机选取4对关键点
        for (int i = 0; i < NUM_OF_PAIR; i++) {
            int idx = random(0, pairs.size() - 1);
            while (selected_indices.find(idx) != selected_indices.end()) {
                idx = random(0, pairs.size() - 1);
            }
            selected_indices.insert(idx);

            random_point_pairs.push_back(pairs[idx]);
        }

        // 计算单应性变换矩阵
        MatrixXf H = createHomography(random_point_pairs);

        // 计算邻域点集合
        vector<int> inliner_set = createInlinerDataSet(pairs, H, selected_indices);

        // 保留邻域点数目最多的单应性变换矩阵的集合
        if (inliner_set.size() > best_inliner.size())
            best_inliner = inliner_set;
    }

    // 对结果进行最小二乘估计
    MatrixXf H = leastSquareSolution(pairs, best_inliner);

    return best_inliner;
}

// 获取 inliners 中关键点的平均偏移量，获取拼接图像时的偏移量
vector<int> Panorama::getImageOffset(const vector<Point>& pairs, vector<int>& inliners) {

    int offset_x = 0;
    int offset_y = 0;
    int min_x = INT_MAX;
    int min_y = INT_MAX;

    int count = 0;
    for (int i = 0; i < inliners.size(); i++) {
        // 计算两个对应关键点之间的偏移量
        int delta_x = pairs[i].a.x - pairs[i].b.x;
        int delta_y = pairs[i].a.y - pairs[i].b.y;
        if (delta_x == 0 || delta_y == 0 || abs(delta_y) > 200) continue;
        offset_x += delta_x;
        offset_y += delta_y;
        count++;

        min_x = (pairs[i].a.x < min_x) ? pairs[i].a.x : min_x;
        min_y = (pairs[i].a.y < min_y) ? pairs[i].a.y : min_y;
    }
    offset_x /= count;
    offset_y /= count;


    int ans_x = 0;
    int ans_y = 0;
    count = 0;
    for (int i = 0; i < inliners.size(); i++) {
        // 计算两个对应关键点之间的偏移量
        int delta_x = pairs[i].a.x - pairs[i].b.x;
        int delta_y = pairs[i].a.y - pairs[i].b.y;

        if (abs(delta_x - offset_x) < abs(offset_x) / 2 && (abs(delta_y - offset_y) < abs(offset_y) / 2 || abs(offset_y) / 2 < 4)) {
            ans_x += delta_x;
            ans_y += delta_y;
            count++;
        }
    }

    if (count != 0) {
        ans_x /= count;
        ans_y /= count;
    }
    
    vector<int> result{ ans_x, ans_y, min_x, min_y };
    return result;
}


// 连接两个图像的算法
CImg<float> Panorama::blendTwoImages(CImg<float>& image1, CImg<float>& image2, int offset_x, int offset_y, int min) {

    // 第一幅图像在第二幅图像前面
    if (offset_x > 0) {
        int nwidth  = image2._width  + abs(offset_x);
        int nheight = image2._height + abs(offset_y);

        CImg<float> result(nwidth, nheight, 1, image2.spectrum(), 0);

        // 将第一幅图像的部分输入结果
        cimg_forXY(image1, i, j) {
            if (i > min) continue;
            for (int k = 0; k < image1.spectrum(); k++) {
                result(i, j, 0, k) = image1(i, j, 0, k);
            }   
        }

        // 将第二幅图像中的相应位置进行偏移后拼接
        cimg_forXY(image2, x, y) {
            for (int k = 0; k < image2.spectrum(); k++)
            result(x + offset_x, y + offset_y, 0, k) = image2(x, y, 0, k);
        }

        return result;
    } 
    
    // 第一幅图像在第二幅图像后面
    else {
        int nwidth = image1._width + abs(offset_x);
        int nheight = image1._height + abs(offset_y);

        CImg<float> result(nwidth, nheight, 1, image1.spectrum(), 0);

        cimg_forXY(image2, i, j) {
            for (int k = 0; k < image2.spectrum(); k++)
            result(i, j, 0, k) = image2(i, j, 0, k);
        }

        cimg_forXY(image1, i, j) {
            if (i < min) continue;
            for (int k = 0; k < image1.spectrum(); k++) {
                result(i - offset_x, j - offset_y, 0, k) = image1(i, j, 0, k);
            }
        }

        return result;
    }
}



CImg<float> Panorama::stitching() {
    bool adjacent[20][20] = { false };
    vector<vector<int>> matching_index(image.size());

    // 获取相邻的图片顺序
    for (int i = 0; i < image.size(); i++) {
        for (int j = i + 1; j < image.size(); j++) {
            vector<Point> pairs = getKeyPointMatch(features[i], features[j]);
			cout << pairs.size() << endl;
            if (pairs.size() >= 20) {
				cout << i << " " << j << endl;
                adjacent[i][j] = true;
                matching_index[i].push_back(j);
            }
        }
    }



    int start = 0;
    int prev = start;
    queue<int> Queue;
    Queue.push(start);
    CImg<float> currentImage = cylinder_image[start];

    while (!Queue.empty()) {
        int sourceIndex = Queue.front();
        Queue.pop();

        for (int i = 0; i < matching_index[sourceIndex].size(); i--) {
            int targetIndex = matching_index[sourceIndex][i];

            if (adjacent[sourceIndex][targetIndex]) {
                adjacent[sourceIndex][targetIndex] = false;
                adjacent[targetIndex][sourceIndex] = false;
                Queue.push(targetIndex);

                // SIFT 获取两幅图像的关键点集
                vector<Point> sourceToTargetPairs = getKeyPointMatch(features[sourceIndex], features[targetIndex]);
                vector<Point> TargetToSourcePairs = getKeyPointMatch(features[targetIndex], features[sourceIndex]);

                if (sourceToTargetPairs.size() > TargetToSourcePairs.size()) {
                    if (!sourceToTargetPairs.empty()) {
                        sourceToTargetPairs.clear();
                    }
                    for (int i = 0; i < TargetToSourcePairs.size(); i++) {
                        sourceToTargetPairs.push_back(Point(TargetToSourcePairs[i].b, TargetToSourcePairs[i].a));
                    }
                }
                    
                else {
                    if (!TargetToSourcePairs.empty()) {
                        TargetToSourcePairs.clear();
                    }
                    for (int i = 0; i < sourceToTargetPairs.size(); i++) {
                        TargetToSourcePairs.push_back(Point(sourceToTargetPairs[i].b, sourceToTargetPairs[i].a));
                    }                    
                }
                
                // 通过RANSAC获取具有最多邻域点的集合
                vector<int> inliners = RANSAC(TargetToSourcePairs);
                // 获取图片的偏移量以用于拼接
                vector<int> params = getImageOffset(sourceToTargetPairs, inliners);
                // 对图片进行相应的拼接
                currentImage = blendTwoImages(currentImage, cylinder_image[targetIndex], params[0], params[1], params[2]);
                currentImage.display("middle", false);

                // prev设置为拼接后的图片
                prev = targetIndex;
            }
        }
    }
    return currentImage;
}


int main(int argc, char* argv[]) {
	string filename;
    int fileNum;
    printf("Please input the file root: ");
    cin >> filename;
    printf("Please input the file num : ");
    cin >> fileNum;

    //string filename = "./TEST1/pano1_";
    Panorama panorama(filename.c_str(), fileNum);
	return 0;
}