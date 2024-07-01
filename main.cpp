////1
//#include <opencv2/opencv.hpp>
//#include <opencv2/dnn.hpp>
//#include <fstream>
//#include <iostream>
//#include <string>
//#include <vector>
//
//// Функция для получения имен классов из файла coco.names
//std::vector<std::string> getOutputsNames(const cv::dnn::Net& net) {
//    static std::vector<std::string> names;
//    if (names.empty()) {
//        std::vector<int> outLayers = net.getUnconnectedOutLayers();
//        std::vector<std::string> layersNames = net.getLayerNames();
//        names.resize(outLayers.size());
//        for (size_t i = 0; i < outLayers.size(); ++i)
//            names[i] = layersNames[outLayers[i] - 1];
//    }
//    return names;
//}
//
//int main() {
//    // Чтение изображения
//    cv::Mat img = cv::imread("720x.jfif");
//    if (img.empty()) {
//        std::cerr << "Error: Image cannot be loaded!" << std::endl;
//        return -1;
//    }
//
//    // Загрузка имен классов
//    std::vector<std::string> classes;
//    std::ifstream ifs("coco.names");
//    std::string line;
//    while (getline(ifs, line)) classes.push_back(line);
//
//    // Загрузка модели YOLO
//    cv::dnn::Net net = cv::dnn::readNetFromDarknet("yolov3.cfg", "yolov3.weights");
//    net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
//    net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
//
//    // Подготовка изображения для сети
//    cv::Mat blob = cv::dnn::blobFromImage(img, 1 / 255.0, cv::Size(416, 416), cv::Scalar(0, 0, 0), true, false);
//    net.setInput(blob);
//
//    // Запуск детектора
//    std::vector<cv::Mat> outs;
//    net.forward(outs, getOutputsNames(net));
//
//    // Обработка результатов
//    float confThreshold = 0.1; // Порог уверенности (уменьшен для захвата большего числа объектов)
//    float nmsThreshold = 0.5;  // Порог подавления нелокальных максимумов
//    std::vector<int> classIds;
//    std::vector<float> confidences;
//    std::vector<cv::Rect> boxes;
//
//    for (size_t i = 0; i < outs.size(); ++i) {
//        float* data = (float*)outs[i].data;
//        for (int j = 0; j < outs[i].rows; ++j, data += outs[i].cols) {
//            cv::Mat scores = outs[i].row(j).colRange(5, outs[i].cols);
//            cv::Point classIdPoint;
//            double confidence;
//            cv::minMaxLoc(scores, 0, &confidence, 0, &classIdPoint);
//            if (confidence > confThreshold) {
//                int centerX = (int)(data[0] * img.cols);
//                int centerY = (int)(data[1] * img.rows);
//                int width = (int)(data[2] * img.cols);
//                int height = (int)(data[3] * img.rows);
//                int left = centerX - width / 2;
//                int top = centerY - height / 2;
//
//                classIds.push_back(classIdPoint.x);
//                confidences.push_back((float)confidence);
//                boxes.push_back(cv::Rect(left, top, width, height));
//            }
//        }
//    }
//    
//    std::vector<int> indices;
//    cv::dnn::NMSBoxes(boxes, confidences, confThreshold, nmsThreshold, indices);
//
//    // Отрисовка прямоугольников вокруг обнаруженных объектов
//    int personCount = 0;
//    for (size_t i = 0; i < indices.size(); ++i) {
//        int idx = indices[i];
//        cv::Rect box = boxes[idx];
//        if (classes[classIds[idx]] == "person") { // фильтруем только людей
//            cv::rectangle(img, box, cv::Scalar(0, 255, 0), 2);
//            personCount++;
//        }
//    }
//
//    // Вывод количества обнаруженных людей на изображении
//    std::string text = "Number of people: " + std::to_string(personCount);
//    int fontFace = cv::FONT_HERSHEY_SIMPLEX;
//    double fontScale = 1;
//    int thickness = 2;
//    cv::putText(img, text, cv::Point(10, 30), fontFace, fontScale, cv::Scalar(0, 0, 255), thickness);
//
//    // Отображение изображения с обнаруженными людьми
//    cv::imshow("Detected People", img);
//    cv::waitKey(0);
//
//    return 0;
//}
//////2
//// Функция для получения имен классов из файла coco.names
//std::vector<std::string> getOutputsNames(const cv::dnn::Net& net) {
//    static std::vector<std::string> names;
//    if (names.empty()) {
//        std::vector<int> outLayers = net.getUnconnectedOutLayers();
//        std::vector<std::string> layersNames = net.getLayerNames();
//        names.resize(outLayers.size());
//        for (size_t i = 0; i < outLayers.size(); ++i)
//            names[i] = layersNames[outLayers[i] - 1];
//    }
//    return names;
//}
//
//int main() {
//    // Чтение изображения
//    cv::Mat img = cv::imread("720x.jfif");
//    if (img.empty()) {
//        std::cerr << "Error: Image cannot be loaded!" << std::endl;
//        return -1;
//    }
//
//    // Загрузка имен классов
//    std::vector<std::string> classes;
//    std::ifstream ifs("coco.names");
//    std::string line;
//    while (getline(ifs, line)) classes.push_back(line);
//
//    // Загрузка модели YOLO
//    cv::dnn::Net net = cv::dnn::readNetFromDarknet("yolov3.cfg", "yolov3.weights");
//    net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
//    net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
//
//    // Подготовка изображения для сети
//    cv::Mat blob = cv::dnn::blobFromImage(img, 1 / 255.0, cv::Size(416, 416), cv::Scalar(0, 0, 0), true, false);
//    net.setInput(blob);
//
//    // Запуск детектора
//    std::vector<cv::Mat> outs;
//    net.forward(outs, getOutputsNames(net));
//
//    // Обработка результатов
//    float confThreshold = 0.1; // Порог уверенности
//    float nmsThreshold = 0.5;  // Порог подавления нелокальных максимумов
//    std::vector<int> classIds;
//    std::vector<float> confidences;
//    std::vector<cv::Rect> boxes;
//
//    for (size_t i = 0; i < outs.size(); ++i) {
//        float* data = (float*)outs[i].data;
//        for (int j = 0; j < outs[i].rows; ++j, data += outs[i].cols) {
//            cv::Mat scores = outs[i].row(j).colRange(5, outs[i].cols);
//            cv::Point classIdPoint;
//            double confidence;
//            cv::minMaxLoc(scores, 0, &confidence, 0, &classIdPoint);
//            if (confidence > confThreshold) {
//                int centerX = (int)(data[0] * img.cols);
//                int centerY = (int)(data[1] * img.rows);
//                int width = (int)(data[2] * img.cols);
//                int height = (int)(data[3] * img.rows);
//                int left = centerX - width / 2;
//                int top = centerY - height / 2;
//
//                classIds.push_back(classIdPoint.x);
//                confidences.push_back((float)confidence);
//                boxes.push_back(cv::Rect(left, top, width, height));
//            }
//        }
//    }
//
//    std::vector<int> indices;
//    cv::dnn::NMSBoxes(boxes, confidences, confThreshold, nmsThreshold, indices);
//
//    // Отрисовка прямоугольников вокруг обнаруженных объектов
//    int personCount = 0;
//    int helmetCount = 0;
//    int noHelmetCount = 0;
//    for (size_t i = 0; i < indices.size(); ++i) {
//        int idx = indices[i];
//        cv::Rect box = boxes[idx];
//        if (classes[classIds[idx]] == "person") { // фильтруем только людей
//            personCount++;
//
//            // Проверка наличия каски (предполагается, что каски оранжевого цвета)
//            cv::Mat personROI = img(box);
//            cv::Mat hsvROI;
//            cv::cvtColor(personROI, hsvROI, cv::COLOR_BGR2HSV);
//            cv::Scalar lower_orange(5, 50, 50); // Нижняя граница цвета каски
//            cv::Scalar upper_orange(15, 255, 255); // Верхняя граница цвета каски
//            cv::Mat mask;
//            cv::inRange(hsvROI, lower_orange, upper_orange, mask);
//            int nonZero = cv::countNonZero(mask);
//            if (nonZero > 100) { // Если количество пикселей каски достаточно, считаем что каска есть
//                helmetCount++;
//                cv::rectangle(img, box, cv::Scalar(255, 0, 0), 2); // Синий прямоугольник для людей в каске
//            }
//            else {
//                noHelmetCount++;
//                cv::rectangle(img, box, cv::Scalar(0, 0, 255), 2); // Красный прямоугольник для людей без каски
//            }
//        }
//    }
//
//    // Вывод количества обнаруженных людей и касок на изображении
//    std::string textPeople = "Number of people: " + std::to_string(personCount);
//    std::string textHelmets = "With helmets: " + std::to_string(helmetCount);
//    std::string textNoHelmets = "Without helmets: " + std::to_string(noHelmetCount);
//    int fontFace = cv::FONT_HERSHEY_SIMPLEX;
//    double fontScale = 1;
//    int thickness = 2;
//    cv::putText(img, textPeople, cv::Point(10, 30), fontFace, fontScale, cv::Scalar(0, 0, 255), thickness);
//    cv::putText(img, textHelmets, cv::Point(10, 60), fontFace, fontScale, cv::Scalar(255, 0, 0), thickness);
//    cv::putText(img, textNoHelmets, cv::Point(10, 90), fontFace, fontScale, cv::Scalar(0, 0, 255), thickness);
//
//    // Отображение изображения с обнаруженными людьми
//    cv::imshow("Detected People", img);
//    cv::waitKey(0);
//
//    return 0;
//}
///////3
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

// Функция для получения имен классов из файла coco.names
std::vector<std::string> getOutputsNames(const cv::dnn::Net& net) {
    static std::vector<std::string> names;
    if (names.empty()) {
        std::vector<int> outLayers = net.getUnconnectedOutLayers();
        std::vector<std::string> layersNames = net.getLayerNames();
        names.resize(outLayers.size());
        for (size_t i = 0; i < outLayers.size(); ++i)
            names[i] = layersNames[outLayers[i] - 1];
    }
    return names;
}

// Функция для нахождения групп людей с помощью DBSCAN
std::vector<std::vector<int>> findGroups(const std::vector<cv::Rect>& boxes, double eps, int minPts) {
    std::vector<std::vector<int>> clusters;
    std::vector<int> labels(boxes.size(), -1);
    int clusterId = 0;

    auto distance = [](const cv::Rect& a, const cv::Rect& b) {
        cv::Point centerA(a.x + a.width / 2, a.y + a.height / 2);
        cv::Point centerB(b.x + b.width / 2, b.y + b.height / 2);
        return cv::norm(centerA - centerB);
    };

    for (size_t i = 0; i < boxes.size(); ++i) {
        if (labels[i] != -1) continue;

        std::vector<int> neighbors;
        for (size_t j = 0; j < boxes.size(); ++j) {
            if (distance(boxes[i], boxes[j]) < eps) {
                neighbors.push_back(j);
            }
        }

        if (neighbors.size() < minPts) {
            labels[i] = -2; // Noise point
            continue;
        }

        clusters.push_back({});
        int currentCluster = clusterId++;
        std::vector<int> queue = neighbors;
        labels[i] = currentCluster;

        while (!queue.empty()) {
            int idx = queue.back();
            queue.pop_back();
            if (labels[idx] == -2) labels[idx] = currentCluster; // Change noise to border point
            if (labels[idx] != -1) continue;

            labels[idx] = currentCluster;
            clusters.back().push_back(idx);

            std::vector<int> newNeighbors;
            for (size_t j = 0; j < boxes.size(); ++j) {
                if (distance(boxes[idx], boxes[j]) < eps) {
                    newNeighbors.push_back(j);
                }
            }

            if (newNeighbors.size() >= minPts) {
                queue.insert(queue.end(), newNeighbors.begin(), newNeighbors.end());
            }
        }
    }

    return clusters;
}

int main() {
    // Чтение изображения
    cv::Mat img = cv::imread("720x.jfif");
    if (img.empty()) {
        std::cerr << "Error: Image cannot be loaded!" << std::endl;
        return -1;
    }

    // Загрузка имен классов
    std::vector<std::string> classes;
    std::ifstream ifs("coco.names");
    std::string line;
    while (getline(ifs, line)) classes.push_back(line);

    // Загрузка модели YOLO
    cv::dnn::Net net = cv::dnn::readNetFromDarknet("yolov3.cfg", "yolov3.weights");
    net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);

    // Подготовка изображения для сети
    cv::Mat blob = cv::dnn::blobFromImage(img, 1 / 255.0, cv::Size(416, 416), cv::Scalar(0, 0, 0), true, false);
    net.setInput(blob);

    // Запуск детектора
    std::vector<cv::Mat> outs;
    net.forward(outs, getOutputsNames(net));

    // Обработка результатов
    float confThreshold = 0.1; // Порог уверенности
    float nmsThreshold = 0.5;  // Порог подавления нелокальных максимумов
    std::vector<int> classIds;
    std::vector<float> confidences;
    std::vector<cv::Rect> boxes;

    for (size_t i = 0; i < outs.size(); ++i) {
        float* data = (float*)outs[i].data;
        for (int j = 0; j < outs[i].rows; ++j, data += outs[i].cols) {
            cv::Mat scores = outs[i].row(j).colRange(5, outs[i].cols);
            cv::Point classIdPoint;
            double confidence;
            cv::minMaxLoc(scores, 0, &confidence, 0, &classIdPoint);
            if (confidence > confThreshold) {
                int centerX = (int)(data[0] * img.cols);
                int centerY = (int)(data[1] * img.rows);
                int width = (int)(data[2] * img.cols);
                int height = (int)(data[3] * img.rows);
                int left = centerX - width / 2;
                int top = centerY - height / 2;

                classIds.push_back(classIdPoint.x);
                confidences.push_back((float)confidence);
                boxes.push_back(cv::Rect(left, top, width, height));
            }
        }
    }

    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confidences, confThreshold, nmsThreshold, indices);

    // Отрисовка прямоугольников вокруг обнаруженных объектов
    std::vector<cv::Rect> personBoxes;
    for (size_t i = 0; i < indices.size(); ++i) {
        int idx = indices[i];
        if (classes[classIds[idx]] == "person") { // фильтруем только людей
            personBoxes.push_back(boxes[idx]);
        }
    }

    // Нахождение групп людей
    double eps = 100.0; // Максимальное расстояние между центрами описывающих рамок
    int minPts = 2; // Минимальное количество людей в группе
    std::vector<std::vector<int>> groups = findGroups(personBoxes, eps, minPts);

    // Отрисовка групп людей
    for (const auto& group : groups) {
        if (group.size() >= minPts) {
            cv::Rect groupRect = personBoxes[group[0]];
            for (size_t i = 1; i < group.size(); ++i) {
                groupRect = groupRect | personBoxes[group[i]];
            }
            cv::rectangle(img, groupRect, cv::Scalar(0, 255, 0), 2);
            std::string text = "Group size: " + std::to_string(group.size());
            cv::putText(img, text, cv::Point(groupRect.x, groupRect.y - 10), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
        }
    }

    // Отображение изображения с обнаруженными группами
    cv::imshow("Detected Groups", img);
    cv::waitKey(0);

    return 0;
}
