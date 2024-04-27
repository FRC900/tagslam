/* -*-c++-*--------------------------------------------------------------------
 * 2019 Bernd Pfrommer bernd.pfrommer@gmail.com
 */

#pragma once

#include "tagslam/camera_intrinsics.h"
#include "tagslam/pose_with_noise.h"
#include <ros/ros.h>
#include <memory>
#include <map>
#include <set>
#include <string>

namespace tagslam {
  class Body; // need forward declaration here
  class Camera {
    using string = std::string;
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // --------- typedefs -------
    typedef std::shared_ptr<Camera>       CameraPtr;
    typedef std::shared_ptr<const Camera> CameraConstPtr;
    typedef std::vector<CameraPtr>        CameraVec;
    // --- getters/setters
    const string &getName()       const { return (name_); }
    const string &getImageTopic() const { return (imageTopic_); }
    const string &getTagTopic()   const { return (tagTopic_); }
    const string &getInfoTopic()  const { return (infoTopic_); }
    const bool   &hasIntrinsics() const { return (has_intrinsics_); }
    const string &getFrameId()    const { return (frameId_); }
    const string &getRigName()    const { return (rigName_); }
    const std::shared_ptr<Body> getRig() const { return (rig_); }
    const PoseNoise &getWiggle() const { return (wiggle_); }
    const CameraIntrinsics& getIntrinsics() const { return (intrinsics_); }
    int  getIndex() const { return (index_); }
    void setRig(const std::shared_ptr<Body> &rig) { rig_ = rig; }
    void setIntrinsics(const CameraIntrinsics &ci) {
      intrinsics_ = ci;
      has_intrinsics_ = true;
    }
    
    // --- static methods
    static CameraPtr parse_camera(const string &name,
                                   XmlRpc::XmlRpcValue config);
    static CameraVec parse_cameras(XmlRpc::XmlRpcValue config);

  private:
    // -------- variables -------
    string                name_;
    int                   index_{-1};
    string                imageTopic_;     // topic for images
    string                tagTopic_;       // topic for tags
    string                frameId_;        // ros frame id of camera
    string                rigName_;        // name of rig body
    std::shared_ptr<Body> rig_;            // pointer to rig body
    CameraIntrinsics      intrinsics_;     // intrinsic calibration
    string                infoTopic_;      // camera info topic
    bool                  has_intrinsics_; // if intrinsics exist yet (if using camera info)
    PoseNoise             wiggle_;         // how rigid the ext calib is
  };

  using CameraPtr      = Camera::CameraPtr;
  using CameraConstPtr = Camera::CameraConstPtr;
  using CameraVec      = Camera::CameraVec;
}
