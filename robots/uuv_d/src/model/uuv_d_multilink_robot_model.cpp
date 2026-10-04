#include <uuv_d/model/uuv_d_multilink_robot_model.h>

namespace uuv_d_model
{
    UUVDMultilinkRobotModel::UUVDMultilinkRobotModel(bool init_with_rosparam, bool verbose, double fc_f_min_thre, double fc_t_min_thre, double epsilon)
        : aerial_robot_model::transformable::RobotModel(init_with_rosparam, verbose, fc_f_min_thre, fc_t_min_thre, epsilon)
    {
        gimbal_rotor_indices_ = {0, 1, 4, 5};  // rotor1, rotor2, rotor5, rotor6
        fixed_rotor_indices_  = {2, 3};        // rotor3, rotor4
        gimbal_parent_links_  = {"link1", "link1", "link3", "link3"};
        gimbal_force_bases_.resize(gimbal_rotor_indices_.size(), Eigen::MatrixXd::Zero(3, 2));
        // Columns are the two force components solved by the allocator.
        // They are chosen to match the gimbal joint axis and the angle recovery
        // raw_angle = atan2(-lambda0, lambda1) in the controller.
        gimbal_force_bases_.at(0) << 0, 0, 1, 0, 0, 1;   // gimbal1 axis +X: +Y, +Z
        gimbal_force_bases_.at(1) << 1, 0, 0, 0, 0, 1;   // gimbal2 axis -Y: +X, +Z
        gimbal_force_bases_.at(2) << -1, 0, 0, 0, 0, 1;  // gimbal3 axis +Y: -X, +Z
        gimbal_force_bases_.at(3) << 0, 0, 1, 0, 0, 1;   // gimbal4 axis +X: +Y, +Z

        rotor_on_gimbal_frame_num_ = gimbal_rotor_indices_.size();
        rotor_on_rigid_frame_num_ = fixed_rotor_indices_.size();
        links_rotation_from_cog_.resize(rotor_on_gimbal_frame_num_+rotor_on_rigid_frame_num_);
        current_gimbal_angles_.resize(rotor_on_gimbal_frame_num_, 0.0);
    }
    void UUVDMultilinkRobotModel::updateRobotModelImpl(const KDL::JntArray& joint_positions)
    {
        aerial_robot_model::transformable::RobotModel::updateRobotModelImpl(joint_positions);

        const auto& seg_tf_map = getSegmentsTf();
        const auto& joint_index_map = getJointIndexMap();
        KDL::Frame cog = getCog<KDL::Frame>();

        for (int i = 0; i < rotor_on_gimbal_frame_num_; ++i)
        {
            const std::string& parent_link = gimbal_parent_links_.at(i);

            KDL::Frame link_f = seg_tf_map.at(parent_link);
            links_rotation_from_cog_.at(i) = (cog.Inverse() * link_f).M;

            std::string gimbal_name = "gimbal" + std::to_string(i + 1);
            auto joint_it = joint_index_map.find(gimbal_name);
            if (joint_it != joint_index_map.end())
             {
                current_gimbal_angles_.at(i) = joint_positions(joint_it->second);
            }
        }
    }
    Eigen::MatrixXd UUVDMultilinkRobotModel::getFullWrenchAllocationMatrixfromCoG()
    {
        Eigen::MatrixXd wrench_matrix = Eigen::MatrixXd::Zero(6, 3 * rotor_on_gimbal_frame_num_);
        Eigen::MatrixXd wrench_map =Eigen::MatrixXd::Zero(6,3);
        wrench_map.block(0,0,3,3) = Eigen::MatrixXd::Identity(3,3);

        KDL::Frame cog = getCog<KDL::Frame>();
        std::vector<KDL::Vector> rotors_origin_from_cog = getRotorsOriginFromCog<KDL::Vector>();
        std::map<int, int> rotor_direction = getRotorDirection();

        int last_col =0;
        for (int i = 0; i < rotor_on_gimbal_frame_num_; i++)
        {
            int rotor_index = gimbal_rotor_indices_.at(i);
            int rotor_number = rotor_index + 1;

            double m_f_rate = getMFRate();

            wrench_map.block(3, 0, 3, 3) =
                aerial_robot_model::skew(
                        aerial_robot_model::kdlToEigen(rotors_origin_from_cog.at(rotor_index)))
                + m_f_rate * rotor_direction.at(rotor_number) * Eigen::MatrixXd::Identity(3, 3);

                wrench_matrix.block(0, last_col, 6, 3) = wrench_map;
            last_col += 3;
        }
        Eigen::MatrixXd integrated_rot = Eigen::MatrixXd::Zero(3*rotor_on_gimbal_frame_num_, 2*rotor_on_gimbal_frame_num_);
        for (int i=0; i < rotor_on_gimbal_frame_num_; i++)
        {
            integrated_rot.block(3*i,2*i,3,2) =
                aerial_robot_model::kdlToEigen(links_rotation_from_cog_.at(i)) *
                gimbal_force_bases_.at(i);
        }
        Eigen::MatrixXd full_q_mat_for_rotors_on_gimbal_frame = wrench_matrix * integrated_rot;
        Eigen::MatrixXd q_mat_for_rotors_on_rigid_frame = getQMatForRotorsOnRigidFrame();
        Eigen::MatrixXd full_q_mat = Eigen::MatrixXd::Zero(6, 2*rotor_on_gimbal_frame_num_ + rotor_on_rigid_frame_num_);
        full_q_mat.block(0,0,6,2*rotor_on_gimbal_frame_num_) = full_q_mat_for_rotors_on_gimbal_frame;
        full_q_mat.block(0,2*rotor_on_gimbal_frame_num_,6,rotor_on_rigid_frame_num_) = q_mat_for_rotors_on_rigid_frame;
        return full_q_mat;

    }
    Eigen::MatrixXd UUVDMultilinkRobotModel::getQMatForRotorsOnRigidFrame()
    {
        std::vector<Eigen::Vector3d> rotors_origin = getRotorsOriginFromCog<Eigen::Vector3d>();
        std::vector<Eigen::Vector3d> rotors_normal = getRotorsNormalFromCog<Eigen::Vector3d>();
        auto& rotor_direction = getRotorDirection();

        Eigen::MatrixXd q_mat = Eigen::MatrixXd::Zero(6, rotor_on_rigid_frame_num_);

        for (int i = 0; i < rotor_on_rigid_frame_num_; i++)
            {
                int rotor_index = fixed_rotor_indices_.at(i);
                int rotor_number = rotor_index + 1;

        double m_f_rate = getMFRate();

        q_mat(0, i) = rotors_normal.at(rotor_index).x();
        q_mat(1, i) = rotors_normal.at(rotor_index).y();
        q_mat(2, i) = rotors_normal.at(rotor_index).z();

        q_mat.block(3, i, 3, 1) =
        rotors_origin.at(rotor_index).cross(rotors_normal.at(rotor_index))
        + m_f_rate * rotor_direction.at(rotor_number) * rotors_normal.at(rotor_index);
    }

  return q_mat;

    }
}
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(uuv_d_model::UUVDMultilinkRobotModel, aerial_robot_model::RobotModel);
