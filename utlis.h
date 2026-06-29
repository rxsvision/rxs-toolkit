#pragma once
#include <queue>
#include <vector>
#include <windows.h>
#include <iostream>
#include <iostream>
#include <string>
#include <locale>
#include <codecvt>
#include "czxDependence/czxTool.h"

#define cp(i) cloud->points[i]
#define difY(i,j) abs(cp(i).y-cp(j).y)

class CircularBuffer
{
public:
    float threshold;
private:
    std::vector<float> buffer;
    int capacity;       // 缓冲区的最大容量
    int count;          // 当前缓冲区中的元素数量
    int head;           // 指向最早插入的数据
    float sum;          // 缓冲区内所有元素的总和

public:
    // 构造函数
    CircularBuffer(int size);
    // 向缓冲区添加元素
    void add(float value);
    float end();
    float average() const;
    void print() const;
};

std::wstring stringToWstring(const std::string& str);
bool clearFolderContents(const std::string& folderPath_in);
std::string getFilenameFromPath(std::string path);
float calculateMean(const std::vector<float>& data);
float calculateStandardDeviation(const std::vector<float>& data);
std::pair<float, float> calculateConfidenceInterval(const std::vector<float>& data);
bool rangeVerify(CP cloud, int i, int range, float threshold);
void writeMatrixToExcel(const Eigen::MatrixXf& matrix, const std::string& filename); // 写入Excel文件的函数
std::vector<Eigen::MatrixXf> splitMatrixByRows(const Eigen::MatrixXf& matrix, int n);
MatrixXf extractColumns(const MatrixXf& inputMatrix, int colIndex1, int colIndex2);
float calculateSlope(const MatrixXf& points);