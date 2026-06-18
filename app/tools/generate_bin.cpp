#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <filesystem>

using namespace std;
using namespace cv;

std::vector<std::string> readImgNames(std::string root) {
	auto img_path_ = root + "mav0/cam0/data/";
    std::cout << "enter " << std::endl;
    std::vector<std::string> img_names;
    std::vector<std::string> names;
    std::filesystem::path folder_path = img_path_;
    std::ofstream ofs(root + "0_save_timestamp.txt");
	int cnt = 0;
    for (const auto& entry : std::filesystem::directory_iterator(folder_path)) {
		cnt++;
		// if (cnt & 1) continue;
        std::string image_name = entry.path().filename().string();
        std::string extension = image_name.substr(image_name.find_last_of(".") + 1);
        if (extension == "jpg" || extension == "jpeg" || extension == "png") {
            img_names.push_back(img_path_ + image_name);
            image_name = image_name.substr(0, image_name.find_last_of("."));
            names.push_back(image_name);
        }
        std::cout << image_name << std::endl;
    }

    std::sort(img_names.begin(), img_names.end());
    std::sort(names.begin(), names.end());
    for(auto name : names){
        ofs << name << std::endl;
    }
    std::cout << img_path_ << " count " << img_names.size() << std::endl;
    return img_names;
}

int main() {
    std::string root = "/ABS/benchmark/imu_to_camera/2023-12-22-60fps-taurus-cam0/";
    auto left = readImgNames(root);
    // auto right = readImgNames(root + "20231213-150fps-fast-morerotations-cam1/mav0/cam0/data/");

    std::ofstream ofs;
    ofs.open(root + "0_img_.bin", ios::binary);

    int cnt = 0;
    for(auto img_name : left){
		cnt++;
		// if (cnt & 1) continue;
        auto img = cv::imread(img_name, cv::IMREAD_GRAYSCALE); 
        ofs.write((char*)img.data, img.total() * img.elemSize());
        if(cnt % 100 == 0){
            std::cout << "0 " << img_name << " " << cnt << std::endl;
        }
    }

    // std::ofstream ofs2;
    // ofs2.open("../1_img_.bin", ios::binary);
    
    // cnt = 0;
    // for(auto img_name : right){
    //     auto img = cv::imread(img_name);
    //     ofs2.write((char*)img.data, img.total() * img.elemSize());
    //     if(cnt++ % 100 == 0){
    //         std::cout << "1 " << img_name << " " << cnt << std::endl;
    //     }
    // }


    return 0;
}

