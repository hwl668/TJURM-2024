#include "threads/pipeline.h"
#include "garage/garage.h"

#include <cmath>
#include <map>
#include <utility>

static std::vector<cv::Point3f>* BigArmorRed3D, *SmallArmorRed3D;
static std::vector<cv::Point3f>* BigArmorBlue3D, *SmallArmorBlue3D;
static bool   plus_pnp_cost_image;
static double plus_pnp_cost_ratio;

void Pipeline::init_locater() {
    auto param = Param::get_instance();

    // 获取装甲板长宽
    float bigArmorRed_width     = (*param)["Points"]["PnP"]["Red"]["BigArmor"]["Width"];
    float bigArmorRed_height    = (*param)["Points"]["PnP"]["Red"]["BigArmor"]["Height"];
    float smallArmorRed_width   = (*param)["Points"]["PnP"]["Red"]["SmallArmor"]["Width"];
    float smallArmorRed_height  = (*param)["Points"]["PnP"]["Red"]["SmallArmor"]["Height"];

    // 获取装甲板长宽
    float bigArmorBlue_width    = (*param)["Points"]["PnP"]["Blue"]["BigArmor"]["Width"];
    float bigArmorBlue_height   = (*param)["Points"]["PnP"]["Blue"]["BigArmor"]["Height"];
    float smallArmorBlue_width  = (*param)["Points"]["PnP"]["Blue"]["SmallArmor"]["Width"];
    float smallArmorBlue_height = (*param)["Points"]["PnP"]["Blue"]["SmallArmor"]["Height"];

    // 获取PnP损失函数参数
    plus_pnp_cost_image = (*param)["Debug"]["PlusPnP"]["CostImage"];

    // 设置装甲板3D坐标，顺序为左上-右上-左下-右下，即矩阵行优先输出顺序
    BigArmorRed3D = new std::vector<cv::Point3f>();
    SmallArmorRed3D = new std::vector<cv::Point3f>();

    BigArmorRed3D->emplace_back(-bigArmorRed_width / 2, -bigArmorRed_height / 2, 0);
    BigArmorRed3D->emplace_back(bigArmorRed_width / 2, -bigArmorRed_height / 2, 0);
    BigArmorRed3D->emplace_back(-bigArmorRed_width / 2, bigArmorRed_height / 2, 0);
    BigArmorRed3D->emplace_back(bigArmorRed_width / 2, bigArmorRed_height / 2, 0);

    SmallArmorRed3D->emplace_back(-smallArmorRed_width / 2, -smallArmorRed_height / 2, 0);
    SmallArmorRed3D->emplace_back(smallArmorRed_width / 2, -smallArmorRed_height / 2, 0);
    SmallArmorRed3D->emplace_back(-smallArmorRed_width / 2, smallArmorRed_height / 2, 0);
    SmallArmorRed3D->emplace_back(smallArmorRed_width / 2, smallArmorRed_height / 2, 0);

    // 设置装甲板3D坐标，顺序为左上-右上-左下-右下，即矩阵行优先输出顺序
    BigArmorBlue3D = new std::vector<cv::Point3f>();
    SmallArmorBlue3D = new std::vector<cv::Point3f>();

    BigArmorBlue3D->emplace_back(-bigArmorBlue_width / 2, -bigArmorBlue_height / 2, 0);
    BigArmorBlue3D->emplace_back(bigArmorBlue_width / 2, -bigArmorBlue_height / 2, 0);
    BigArmorBlue3D->emplace_back(-bigArmorBlue_width / 2, bigArmorBlue_height / 2, 0);
    BigArmorBlue3D->emplace_back(bigArmorBlue_width / 2, bigArmorBlue_height / 2, 0);

    SmallArmorBlue3D->emplace_back(-smallArmorBlue_width / 2, -smallArmorBlue_height / 2, 0);
    SmallArmorBlue3D->emplace_back(smallArmorBlue_width / 2, -smallArmorBlue_height / 2, 0);
    SmallArmorBlue3D->emplace_back(-smallArmorBlue_width / 2, smallArmorBlue_height / 2, 0);
    SmallArmorBlue3D->emplace_back(smallArmorBlue_width / 2, smallArmorBlue_height / 2, 0);
}

namespace {

// yaw 差归一化到 [-pi, pi]
double normalize_angle(double a) {
    while (a > M_PI)  a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

// IPPE 消歧记忆：同 (camera_id, armor_id) 上一帧的 armor yaw（world 系）。
// 键数量上界 = 相机数 × 装甲板 ID 数，无需清理。
std::map<std::pair<int, int>, double> last_armor_yaw;

}  // namespace

bool Pipeline::locater(std::shared_ptr<rm::Frame> frame) {
    auto garage = Garage::get_instance();

    Eigen::Vector4d pose_world;

    Eigen::Matrix3d rotate_pnp2head, rotate_head2world;
    Eigen::Matrix4d trans_pnp2head, trans_head2world;

    rm::Camera* camera = Data::camera[frame->camera_id];

    rotate_pnp2head = camera->Rotate_pnp2head;
    rm::tf_rotate_head2world(rotate_head2world, frame->yaw, frame->pitch, frame->roll);

    trans_pnp2head = camera->Trans_pnp2head;
    rm::tf_trans_head2world(trans_head2world, frame->yaw, frame->pitch, frame->roll);


    for(auto& armor : frame->armor_list) {
        if(armor.four_points.size() != 4) { 
            continue;
        }

        auto objptr = garage->getObj(armor.id);
        rm::ArmorSize obj_size = objptr->getArmorSize();
        rm::ArmorSize curr_size =
            (obj_size == rm::ARMOR_SIZE_UNKNOWN) ? armor.size : obj_size;

        std::vector<cv::Point3f>* Armor3D = nullptr;
        if(curr_size == rm::ARMOR_SIZE_BIG_ARMOR) {
            if(armor.color == rm::ARMOR_COLOR_RED) Armor3D = BigArmorRed3D;
            else if(armor.color == rm::ARMOR_COLOR_BLUE) Armor3D = BigArmorBlue3D;
            else continue;
        } else if(curr_size == rm::ARMOR_SIZE_SMALL_ARMOR) {
            if(armor.color == rm::ARMOR_COLOR_RED) Armor3D = SmallArmorRed3D;
            else if(armor.color == rm::ARMOR_COLOR_BLUE) Armor3D = SmallArmorBlue3D;
            else continue;
        } else {
            continue;
        }

        rm::Target target;
        target.armor_id = armor.id;
        target.armor_size = armor.size;

        if (Data::plus_pnp) {
            target.armor_yaw_world = rm::solveYawPnP(
                frame->yaw, camera, pose_world, *Armor3D, armor.four_points, 
                rotate_head2world, trans_head2world, armor.id, plus_pnp_cost_image);
            target.pose_world = pose_world;
            
        } else {
            // IPPE 对平面矩形存在镜像二义性：装甲板接近正对相机时，两解的
            // 重投影误差接近，solvePnP 只返回误差较小解，yaw 会随帧随机翻转
            // （±几十度），污染反陀螺/EKF 输入。这里用 solvePnPGeneric 取出
            // 全部解，按“同相机同 ID 上一帧 yaw 连续性”择优：
            //   首次观测 → 取重投影误差最小的解；
            //   有历史   → 取与上一帧 yaw 最接近的解（正确率远高于纯误差序）。
            std::vector<cv::Mat> sol_rvecs, sol_tvecs;
            std::vector<float> sol_errors;
            int sol_cnt = 0;
            try {
                sol_cnt = cv::solvePnPGeneric(
                    *Armor3D, armor.four_points,
                    Data::camera[frame->camera_id]->intrinsic_matrix,
                    Data::camera[frame->camera_id]->distortion_coeffs,
                    sol_rvecs, sol_tvecs, false, cv::SOLVEPNP_IPPE,
                    cv::noArray(), cv::noArray(), sol_errors);
            } catch (const cv::Exception& e) {
                rm::message("solvePnP error", rm::MSG_ERROR);
                continue;
            }
            if (sol_cnt <= 0) continue;

            std::vector<double> sol_yaw(sol_cnt);
            std::vector<Eigen::Vector4d> sol_pose(sol_cnt);
            for (int i = 0; i < sol_cnt; ++i) {
                cv::Mat rotate_cv_i;
                cv::Rodrigues(sol_rvecs[i], rotate_cv_i);
                Eigen::Matrix3d rotate_pnp_i;
                rm::tf_Mat3d(rotate_cv_i, rotate_pnp_i);
                const Eigen::Matrix3d rotate_world_i =
                    rotate_head2world * rotate_pnp2head * rotate_pnp_i;
                sol_yaw[i] = rm::tf_rotation2armoryaw(rotate_world_i);

                Eigen::Vector4d pose_pnp_i;
                rm::tf_Vec4d(sol_tvecs[i], pose_pnp_i);
                sol_pose[i] = trans_head2world * trans_pnp2head * pose_pnp_i;
            }

            const std::pair<int, int> key(frame->camera_id,
                                          static_cast<int>(armor.id));
            auto it = last_armor_yaw.find(key);
            int best = 0;
            if (it != last_armor_yaw.end()) {
                double best_d = 1e9;
                for (int i = 0; i < sol_cnt; ++i) {
                    const double d = std::fabs(normalize_angle(sol_yaw[i] - it->second));
                    if (d < best_d) { best_d = d; best = i; }
                }
            } else {
                double best_e = 1e9;
                for (int i = 0; i < sol_cnt; ++i) {
                    if (sol_errors[i] < best_e) { best_e = sol_errors[i]; best = i; }
                }
            }
            last_armor_yaw[key] = sol_yaw[best];

            target.armor_yaw_world = sol_yaw[best];
            target.pose_world = sol_pose[best];
        }
        
        frame->target_list.push_back(target);

        double distance = sqrt(pow(target.pose_world(0), 2) + pow(target.pose_world(1), 2) + pow(target.pose_world(2), 2));
        rm::message("pnp dist", distance);
        rm::message("pnp yaw", target.armor_yaw_world * (180 / M_PI));
    }

    if(frame->target_list.size() == 0) {
        if (Data::image_flag) imshow(frame);
        return false;
    }
    
    return true;
}