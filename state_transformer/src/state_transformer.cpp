#include <ros/ros.h>
#include <planning_environment/models/model_utils.h>
#include <string>

#include "state_transformer/GetTransform.h"
#include "state_transformer/GetRobotMarker.h"

class StateTransformer {
public:
  planning_environment::RobotModels robot_model;

  StateTransformer() : robot_model("/robot_description") {}

  bool transform(state_transformer::GetTransform::Request &req,
		 state_transformer::GetTransform::Response &res) {
    
    planning_models::KinematicState kstate(robot_model.getKinematicModel());
    
    if (!planning_environment::setRobotStateAndComputeTransforms
	(req.robot_state, kstate)) {
      ROS_ERROR("Unable to transform robot state to kinematic state");
      res.val = res.BAD_ROBOT_STATE;
      return true;
    }

    std::string from_frame = relative_frame(req.from_frame_id);
    std::string to_frame = relative_frame(req.to_frame_id);

    ROS_INFO("Transforming from %s to %s", from_frame.c_str(), to_frame.c_str());
    
    std::vector<geometry_msgs::TransformStamped> transforms;
    btTransform global_to_to, global_to_from;
    if (!world_transform(from_frame, kstate, global_to_from)) {
      res.val = res.UNABLE_TO_FIND_FROM_FRAME;
      return true;
    }
    if (!world_transform(to_frame, kstate, global_to_to)) {
      res.val = res.UNABLE_TO_FIND_TO_FRAME;
      return true;
    }
    
    //This is the transform that will take the origin of to
    //to the origin of from.  This is how TF works so we are sticking
    //with that convention although it is confusing.  The transform
    //with from=robot frame to=wrist frame will give you the position of
    //the wrist in the robot frame for example.
    //
    //HOWEVER, the transform that takes a pose expressed in from and transforms
    //it to a pose expressed in to is the INVERSE of this transform
    btTransform trans = (global_to_from.inverse())*global_to_to;
    tf::transformTFToMsg(trans, res.transform_stamped.transform);
    res.transform_stamped.header.stamp = ros::Time(0);
    res.transform_stamped.header.frame_id = req.from_frame_id;
    res.transform_stamped.child_frame_id = req.to_frame_id;
    res.val = res.SUCCESS;
    return true; 
  }

  bool get_robot_marker(state_transformer::GetRobotMarker::Request &req,
			state_transformer::GetRobotMarker::Response &res) {

    planning_models::KinematicState state(robot_model.getKinematicModel());
      
    if (!planning_environment::setRobotStateAndComputeTransforms
	(req.robot_state, state)) {
      ROS_ERROR("Unable to transform robot state to kinematic state");
      return true;
    }

    
    boost::shared_ptr<urdf::Model> robot_model_urdf = robot_model.getParsedDescription();

    std::vector<std::string> link_names;

    if(!req.link_names.size()) {
      robot_model.getKinematicModel()->getLinkModelNames(link_names);
    } else {
      link_names = req.link_names;
    }

    for(unsigned int i = 0; i < link_names.size(); i++) {
      boost::shared_ptr<const urdf::Link> urdf_link = robot_model_urdf->getLink(link_names[i]);

      if(!urdf_link) {
	ROS_INFO_STREAM("Invalid urdf name " << link_names[i]);
	continue;
      }

      if(!urdf_link->collision && !urdf_link->visual) {
	continue;
      }
      
      const urdf::Geometry *geom = NULL;
      if (urdf_link->collision) {
	geom = urdf_link->collision->geometry.get();
      } else if(urdf_link->visual) {
	geom = urdf_link->visual->geometry.get();
      }

      if(!geom) {
	continue;
      }

      const urdf::Mesh *mesh = dynamic_cast<const urdf::Mesh*> (geom);
      const urdf::Box *box = dynamic_cast<const urdf::Box*> (geom);
      const urdf::Sphere *sphere = dynamic_cast<const urdf::Sphere*> (geom);
      const urdf::Cylinder *cylinder = dynamic_cast<const urdf::Cylinder*> (geom);


      const planning_models::KinematicState::LinkState* ls = state.getLinkState(link_names[i]);

      if(ls == NULL) {
	ROS_WARN_STREAM("No link state for name " << link_names[i] << " though there's a mesh");
	continue;
      }

      visualization_msgs::Marker mark;
      mark.header.frame_id = robot_model.getWorldFrameId();
      mark.header.stamp = ros::Time::now();
      mark.ns = req.ns;
      mark.id = i;
      tf::poseTFToMsg(ls->getGlobalCollisionBodyTransform(), mark.pose);
      mark.color = req.color;
      
      if(mesh) {
	if(mesh->filename.empty()) {
	  continue;
	}
	mark.type = mark.MESH_RESOURCE;
	mark.scale.x = mesh->scale.x*req.scale;
	mark.scale.y = mesh->scale.y*req.scale;
	mark.scale.z = mesh->scale.z*req.scale;
	mark.mesh_resource = mesh->filename;
      } else if(box) {
	mark.type = mark.CUBE;
	mark.scale.x = box->dim.x;
	mark.scale.y = box->dim.y;
	mark.scale.z = box->dim.z;
      } else if(cylinder) {
	mark.type = mark.CYLINDER;
	mark.scale.x = cylinder->radius;
	mark.scale.y = cylinder->radius;
	mark.scale.z = cylinder->length;
      } else if(sphere) {
	mark.type = mark.SPHERE;
	mark.scale.x = mark.scale.y = mark.scale.z = sphere->radius;
      } else {
	ROS_WARN_STREAM("Unknown object type for link " << link_names[i]);
	continue;
      }
      res.marker_array.markers.push_back(mark);
    }
    return true;
  }
 
protected:

  std::string relative_frame(std::string frame_id) {
    if (frame_id[0] != '/') {
      return frame_id;
    }
    return frame_id.substr(1, frame_id.size());
  }
  

  bool world_transform(std::string frame_id, 
		       const planning_models::KinematicState &state,
		       btTransform &transform) {
    if (!frame_id.compare(state.getKinematicModel()->getRoot()->getParentFrameId())) {
      //identity transform
      transform.setIdentity();
      return true;
    }

    if (!frame_id.compare(state.getKinematicModel()->getRoot()->getChildFrameId())) {
      transform = state.getRootTransform();
      return true;
    }

    const planning_models::KinematicState::LinkState *link =
      state.getLinkState(frame_id);
    if (!link) {
      ROS_ERROR("Unable to find link %s in kinematic state", frame_id.c_str());
      return false;
    }
    
    transform = link->getGlobalLinkTransform();
    return true;
  }			   

};

int main(int argc, char **argv) {
  ros::init(argc, argv, "state_transformer_server");
  ros::NodeHandle n;

  StateTransformer transformer;

  ros::ServiceServer service = n.advertiseService
    ("get_state_transforms", &StateTransformer::transform, &transformer);

  ros::ServiceServer marker_service = n.advertiseService
    ("get_robot_marker", &StateTransformer::get_robot_marker, &transformer);

  ROS_INFO("Ready to transform states.  World frame is %s, robot frame is %s.",
	   transformer.robot_model.getKinematicModel()->getRoot()->getParentFrameId().c_str(),
	   transformer.robot_model.getKinematicModel()->getRoot()->getChildFrameId().c_str());
  ros::spin();
  return 0;
}
