#pragma once

#include <aerial_robot_model/model/transformable_aerial_robot_model.h>
#include <kdl/frames.hpp>
#include <Eigen/Dense>

namespace uuv_d_model
{
    class UUVDMultilinkRobotModel : public aerial_robot_model::transformable::RobotModel
    {
        public:
        UUVDMultilinkRobotModel(bool init_with_rosparam =true, bool verbose = false, double fc_f_min_thre=0.0, double fc_t_min_thre=0.0, double epsilon =0.1);
        virtual ~UUVDMultilinkRobotModel() = default;

        Eigen::MatrixXd getFullWrenchAllocationMatrixfromCoG();
        const std::vector<double>& getCurrentGimbalAngles() const { return current_gimbal_angles_; }
        int getRotorOnRigitFrameNum() const { return rotor_on_rigid_frame_num_; }
        int getRotorOnGimbalFrameNum() const { return rotor_on_gimbal_frame_num_; }
        const std::vector<int>& getGimbalRotorIndices() const { return gimbal_rotor_indices_; }
        const std::vector<int>& getFixedRotorIndices() const { return fixed_rotor_indices_; }
        protected:
        void updateRobotModelImpl(const KDL::JntArray& joint_positions) override;
        private:
        int rotor_on_rigid_frame_num_;
        int rotor_on_gimbal_frame_num_;
        std::vector<int> gimbal_rotor_indices_;
        std::vector<int> fixed_rotor_indices_;
        std::vector<std::string> gimbal_parent_links_;
        std::vector<Eigen::MatrixXd> gimbal_force_bases_;

        std::vector<KDL::Rotation> links_rotation_from_cog_;
        std::vector<double> current_gimbal_angles_;
        Eigen::MatrixXd getQMatForRotorsOnRigidFrame();


    };
}