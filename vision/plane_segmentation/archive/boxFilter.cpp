void PlaneSegmentation::boxFilter()
{
  // convert box to 3D
  std::vector<Eigen::Vector4d> boxes_cam;
  for (int i = 0; i < boxes_.size(); ++i)
  {
    Eigen::Vector3d curr_box_min_pixel(boxes_[i][0], boxes_[i][1], 1);
    Eigen::Vector3d curr_box_max_pixel(boxes_[i][2], boxes_[i][3], 1);

    Eigen::Vector3d curr_box_min_cam;
    Eigen::Vector3d curr_box_max_cam;

    curr_box_min_cam = K_ * curr_box_min_pixel;
    curr_box_max_cam = K_ * curr_box_max_pixel;

    Eigen::Vector4d curr_box_cam(curr_box_min_cam[0], curr_box_min_cam[1], curr_box_max_cam[0], curr_box_max_cam[1]);
    boxes_cam.push_back(curr_box_cam);
  }

  for ()
}