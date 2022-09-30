#ifndef BEZIER_PAINTING_HPP
#define BEZIER_PAINTING_HPP

#include <bezier/bezier_library.hpp>
#include <ros/package.h>
#include <memory>


/**
 * Bezier painting class
 */
class BezierPainting : public Bezier {

public:

  enum INPUT_MESHES
  {
    PAINTING_MESH
  };

  BezierPainting(const std::string input_mesh,
                 const double painting_cone_width,
                 const unsigned covering_percentage,
                 const double extrication_radius,
                 const double lean_angle = 0.0,
                 const Bezier::AXIS_OF_ROTATION axis_of_rotation = Bezier::AXIS_OF_ROTATION::Y,
                 const Eigen::Vector3d &slicing_orientation = Eigen::Vector3d::Zero());

  BezierPainting(const std::string input_mesh);

  virtual ~BezierPainting()
  {
  }

  std::string name()
  {
    return "BezierPainting";
  }

  /**
   * Allows to publish the input mesh and the dilated mesh
   * @param input_mesh_publisher
   * @param dilated_mesh_publisher
   * @note If not called, no mesh will be displayed
   */
  void setMeshesPublishers(std::shared_ptr<ros::Publisher> &input_mesh_publisher,
                           std::shared_ptr<ros::Publisher> &dilated_mesh_publisher);

  std::string generateTrajectory(EigenSTL::vector_Isometry3d &trajectory,
                                 std::vector<bool> &is_grinding_pose,
                                 const bool display_markers = true);

  std::string generateTrajectory(EigenSTL::vector_Isometry3d &trajectory,
                                 std::vector<bool> &is_grinding_pose,
                                 const double painting_cone_width,
                                 const unsigned covering_percentage,
                                 const double extrication_radius,
                                 const double lean_angle = 0.0,
                                 const Bezier::AXIS_OF_ROTATION axis_of_rotation = Bezier::AXIS_OF_ROTATION::Y,
                                 const Eigen::Vector3d &slicing_orientation = Eigen::Vector3d::Zero(),
                                 const bool display_markers = true)
  {
    painting_cone_width_ = painting_cone_width;
    covering_percentage_ = covering_percentage;
    extrication_radius_ = extrication_radius;
    axis_of_rotation_ = axis_of_rotation;
    lean_angle_ = lean_angle;
    setSlicingOrientation(slicing_orientation);
    return (generateTrajectory(trajectory, is_grinding_pose, display_markers));
  }

  /**
   * Force the cutting plane orientation, when set the automatic cutting orientation estimation is skipped.\n
   * The orientation provided is normalized.
   * @param[in] cutting_plane_normal
   * @note To restore the automatic estimation behavior, use setAutomaticCuttingOrientationEstimation
   */
  void setSlicingOrientation(const Eigen::Vector3d &cutting_plane_normal);

private:
  std::string
  validateParameters();

  // Parameters
  double painting_cone_width_;
  unsigned covering_percentage_;
  double extrication_radius_;
  double lean_angle_;
  Bezier::AXIS_OF_ROTATION axis_of_rotation_;

  // Internals
  Eigen::Vector3d slicing_orientation_;
  std::string input_mesh_absolute_path_;
  std::shared_ptr<ros::Publisher> input_mesh_pub_;
  std::shared_ptr<ros::Publisher> dilated_mesh_pub_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif
