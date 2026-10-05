#include "threads/pipeline.h"
#include <string>
#include <thread>
#include <chrono>

static std::string imshow_msg_buffer;

void Pipeline::imshow(std::shared_ptr<rm::Frame> frame_show) {
    this->imshow_register_ = frame_show;
    this->imshow_in_ = true;
}

void Pipeline::imshow(std::shared_ptr<rm::Frame> frame_show, std::string& frame_msg) {
    this->imshow_register_ = frame_show;
    imshow_msg_buffer = frame_msg;
    this->imshow_in_ = true;
}

void Pipeline::image_thread() {
    auto param = Param::get_instance();
    auto garage = Garage::get_instance();

    std::string imwrite_dir;
    imwrite_dir = (*param)["Camera"]["DebugSaveDir"];
    int fps = (*param)["Debug"]["ImageThread"]["FPS"];
    bool light_flag = (*param)["Debug"]["ImageThread"]["Light"];
    double scale = (*param)["Debug"]["ImageThread"]["Scale"];

    int delay = 1000 / fps;

    while(true) {
        // 原实现是 while(!imshow_in_) {} 热旋，独占整核；改为 1ms 休眠轮询
        if (!imshow_in_) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        cv::Mat image = *(this->imshow_register_->image);
        // 先清旗标再处理：空帧/类型异常的 continue 不再原地死循环刷屏
        imshow_in_ = false;

        if (image.empty()) {
            std::cout << "图像为空，无法显示" << std::endl;
            continue;
        } else if (image.type() != CV_8UC1 && image.type() != CV_8UC3 && image.type() != CV_32FC1 && image.type() != CV_32FC3) {
            std::cout << "图像类型不受支持，无法显示" << std::endl;
            continue;
        } else {
            cv::Mat resized_image;
            cv::resize(image, resized_image, cv::Size(image.cols * scale, image.rows * scale));

            if (light_flag) {
                std::vector<cv::Mat> channels;
                cv::split(resized_image, channels);
                cv::equalizeHist(channels[0], channels[0]);
                cv::equalizeHist(channels[1], channels[1]);
                cv::equalizeHist(channels[2], channels[2]);
                cv::merge(channels, resized_image);
            }


            if (Data::imshow_flag) {
                cv::imshow("tjurm2024frame", resized_image);
                cv::waitKey(1);
            }

            if (Data::imwrite_flag) {
                std::string path = imwrite_dir + "/" + getMsStr() + ".jpg";
                std::vector<std::string> lines;
                if(Data::target_id != rm::ARMOR_ID_UNKNOWN) {
                    auto obj = garage->getObj(Data::target_id);
                    obj->getState(lines);
                }
                rm::displayStrVecOnImage(resized_image, lines);
                cv::imwrite(path, resized_image);
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(delay));
        }

    }
}