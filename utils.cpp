#include"utlis.h"


CircularBuffer::CircularBuffer(int size) : capacity(size), count(0), head(0), sum(0.0f), threshold(0.2f) {
    buffer.resize(capacity);
}

// 向缓冲区添加元素
void CircularBuffer::add(float value) {
    if (count != 0 && abs(end() - value) > threshold)
    {
        buffer.clear();
        count = 0;
        head = 0;
        sum = 0;
    }
    if (count < capacity) {
        buffer[count] = value;
        sum += value;
        count++;
    }
    else {
        // 当缓冲区已满，替换最早的元素
        sum -= buffer[head];
        buffer[head] = value;
        sum += value;
        head = (head + 1) % capacity; // 移动头指针
    }
}

float CircularBuffer::end()
{
    if (count == 0) return 0;
    return buffer[count - 1];
}

// 计算缓冲区内所有元素的平均值
float CircularBuffer::average() const {
    if (count == 0) return 0.0;  // 防止除以零
    return sum / count;
}

// 打印缓冲区的内容（调试用）
void CircularBuffer::print() const {
    std::cout << "Buffer contents: ";
    for (int i = 0; i < count; ++i) {
        int index = (head + i) % capacity;
        std::cout << buffer[index] << " ";
    }
    std::cout << std::endl;
}


// 将 std::string 转换为 std::wstring
std::wstring stringToWstring(const std::string& str) {
    if (str.empty()) return L"";
    int size_needed = MultiByteToWideChar(CP_UTF8, 0, &str[0], (int)str.size(), NULL, 0);
    std::wstring wstrTo(size_needed, 0);
    MultiByteToWideChar(CP_UTF8, 0, &str[0], (int)str.size(), &wstrTo[0], size_needed);
    return wstrTo;
}

bool clearFolderContents(const std::string& folderPath_in) {
    std::wstring folderPath = stringToWstring(folderPath_in);
    WIN32_FIND_DATA findFileData;
    HANDLE hFind = FindFirstFile((folderPath + L"\\*").c_str(), &findFileData);

    if (hFind == INVALID_HANDLE_VALUE) {
        std::wcerr << L"Failed to access folder: " << folderPath << std::endl;
        return false;
    }

    do {
        std::wstring filename = findFileData.cFileName;
        if (filename == L"." || filename == L"..") {
            continue;
        }

        std::wstring fullPath = folderPath + L"\\" + filename;

        if (!(findFileData.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY)) {
            if (!DeleteFile(fullPath.c_str())) {
                std::wcerr << L"Failed to delete file: " << fullPath << std::endl;
                FindClose(hFind);
                return false;
            }
        }
    } while (FindNextFile(hFind, &findFileData));

    FindClose(hFind);
    return true;
}

std::string getFilenameFromPath(std::string path) {
    if (!path.empty() && (path.back() == '/' || path.back() == '\\')) {
        path.pop_back();
    }
    size_t pos = path.find_last_of("/\\");
    if (pos != std::string::npos) {
        return path.substr(pos + 1);
    }
    return path; // 如果没有找到斜杠或反斜杠，返回整个字符串
}

float calculateMean(const std::vector<float>& data) {
    float sum = 0.0f;
    for (float num : data) {
        sum += num;
    }
    return sum / data.size();
}

float calculateStandardDeviation(const std::vector<float>& data) {
    float mean = calculateMean(data);
    float sumSquaredDiffs = 0.0f;
    for (float num : data) {
        float diff = num - mean;
        sumSquaredDiffs += diff * diff;
    }
    float variance = sumSquaredDiffs / data.size();
    return std::sqrt(variance);
}

std::pair<float, float> calculateConfidenceInterval(const std::vector<float>& data) {
    float mean = calculateMean(data);
    float standardDeviation = calculateStandardDeviation(data);
    float criticalValue = 1.96f; // 95% 置信度对应的临界值
    float marginError = criticalValue * (standardDeviation / std::sqrt(data.size()));
    float lowerBound = mean - marginError;
    float upperBound = mean + marginError;
    cout << "置信区间是:" << lowerBound << "-" << upperBound << endl;
    return std::make_pair(lowerBound, upperBound);
}

bool rangeVerify(CP cloud, int i, int range, float threshold)
{
    if (i < range)
    {
        int off = 1;
        while (off <= range)
        {
            if (difY(i, i + off) > threshold)
                return false;
            off++;
        }
        return true;
    }

    if (i > cloud->size() - range - 1)
    {
        int off = 1;
        while (off <= range)
        {
            if (difY(i, i - off) > threshold)
                return false;
            off++;
        }
        return true;
    }

    {
        bool verify_left = true;
        int off = 1;
        while (off <= range)
        {
            if (difY(i, i + off) > threshold)
            {
                verify_left = false;
                break;
            }
            off++;
        }
        if (verify_left) return true;


        bool verify_right = true;
        while (off <= range)
        {
            if (difY(i, i - off) > threshold)
            {
                verify_right = false;
                break;
            }
            off++;
        }
        return verify_right;
    }
}

// 写入Excel文件的函数
void writeMatrixToExcel(const Eigen::MatrixXf& matrix, const std::string& filename)
{
    // 打开文件流
    std::ofstream file(filename);

    // 检查文件是否成功打开
    if (!file.is_open()) {
        std::cerr << "Error: Failed to open file " << filename << std::endl;
        return;
    }

    // 写入矩阵数据到文件
    for (int i = 0; i < matrix.rows(); ++i) {
        for (int j = 0; j < matrix.cols(); ++j) {
            file << matrix(i, j);
            if (j < matrix.cols() - 1) {
                file << ","; // 使用逗号分隔数据
            }
        }
        file << std::endl; // 换行
    }

    // 关闭文件流
    file.close();

    std::cout << "Matrix has been written to " << filename << std::endl;
}

std::vector<Eigen::MatrixXf> splitMatrixByRows(const Eigen::MatrixXf& matrix, int n)
{
    std::vector<Eigen::MatrixXf> result;

    if (matrix.rows() % n != 0) 
    {
        std::cerr << "Error: Matrix rows cannot be evenly split into " << n << " submatrices." << std::endl;
        return result;
    }

    int rowsPerSubMatrix = matrix.rows() / n; // 每个子矩阵的行数

    for (int i = 0; i < n; ++i) {
        // 提取子矩阵
        Eigen::MatrixXf subMatrix(rowsPerSubMatrix, matrix.cols());
        result.push_back(subMatrix);
    }

    for (int i = 0; i < matrix.rows(); i++)
    {
        result[i % n].row(i / n) = matrix.row(i);
    }

    //for (auto& mat : result)
    //{
    //    cout << mat << endl;
    //}

    return result;
}

MatrixXf extractColumns(const MatrixXf& inputMatrix, int colIndex1, int colIndex2)
{
    // 检查列索引是否有效
    if (colIndex1 < 0 || colIndex2 < 0 || colIndex1 >= inputMatrix.cols() || colIndex2 >= inputMatrix.cols()) {
        std::cerr << "Invalid column index!" << std::endl;
        exit(1);
    }

    // 创建一个新的矩阵来存储提取的列
    MatrixXf extractedMatrix(inputMatrix.rows(), 2);

    // 提取对应列
    extractedMatrix.col(0) = inputMatrix.col(colIndex1);
    extractedMatrix.col(1) = inputMatrix.col(colIndex2);

    return extractedMatrix;
}

float calculateSlope(const MatrixXf& points) 
{
    // 计算数据点的均值
    float sum_x = points.col(0).sum();
    float sum_y = points.col(1).sum();
    float mean_x = sum_x / points.rows();
    float mean_y = sum_y / points.rows();

    // 计算斜率的分子和分母
    float numerator = 0.0;
    float denominator = 0.0;
    for (int i = 0; i < points.rows(); ++i) {
        float dx = points(i, 0) - mean_x;
        float dy = points(i, 1) - mean_y;
        numerator += dx * dy;
        denominator += dx * dx;
    }

    // 计算斜率
    if (denominator == 0.0) {
        cerr << "Cannot compute slope: denominator is zero!" << endl;
        exit(1);
    }
    float slope = numerator / denominator;
    return slope;
}