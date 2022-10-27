// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Luning Fang
// =============================================================================
//
// scorbot model
// irrlicht visualize with global y pointing up, transformation needed 
// todo: driver input actuation file
// =============================================================================

#include "chrono/core/ChStream.h"
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"
#include "chrono/assets/ChTriangleMeshShape.h"



#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono_thirdparty/filesystem/path.h"

#include <cmath>
using namespace chrono;
using namespace chrono::irrlicht;
using namespace irr;

collision::ChCollisionSystemType collision_type = collision::ChCollisionSystemType::BULLET;

enum class VisualizationType{
    CYLINDER,
    BOX,
    MESH
};






// Convert a triplet (roll-pitch-yaw) to a quaternion
ChQuaternion<> rpy2quat(const ChVector<>& rpy) {
    return Q_from_AngZ(rpy.z()) * Q_from_AngY(rpy.y()) * Q_from_AngX(rpy.x());
}

// link data from urdf file (relative coordinate formulation)
struct LinkData{
    std::string name;
    ChVector<> origin_xyz;
    ChVector<> origin_rpy;
    ChVector<> dim;   // size, for dynamics and primitive visualization
    std::string meshname;
};

// joint data from urdf file (relative coordinate formulation)
struct JointData{
    std::string name;
    std::string link_parent;
    std::string link_child;
    ChVector<> origin_xyz;
    ChVector<> origin_rpy;
    ChVector<> axis;
    double lower_limit;
    double upper_limit;
};

const int numLinks = 8;
const int numJoints = 7;

LinkData scorbot_links[] = {
    {"link0", ChVector<>(0.27, 0.469, 0), ChVector<>(0, 0, 0), ChVector<>(0.24, 0.24, 0.2274), "base_Link"},
    {"link1", ChVector<>(0, 0.075, 0),    ChVector<>(0, 0, 0), ChVector<>(0.205, 0.26, 0.17), "shoulder_Link"},
    {"link2", ChVector<>(0.11, 0, 0),     ChVector<>(0, 0, 0), ChVector<>(0.28749, 0.069982, 0.169), "elbow_Link"},
    {"link3", ChVector<>(0.12, 0, 0),     ChVector<>(0, 0, 0), ChVector<>(0.289918, 0.06, 0.15), "pitch_Link"},
    {"link4", ChVector<>(-0.04, 0, 0),    ChVector<>(0, 0, 0), ChVector<>(0.183046, 0.073212, 0.075543), "roll_Link"},
    {"link5", ChVector<>(0, 0.02, 0),    ChVector<>(0, 0, 0), ChVector<>(0.06, 0.14, 0.238), "gripper_Link"},
    {"pad1_link", ChVector<>(-0.035, 0, 0),    ChVector<>(0, 0, 0), ChVector<>(0.073966, 0.01, 0.02), "pad1_Link"},
    {"pad2_link", ChVector<>(-0.035, 0, 0),    ChVector<>(0, 0, 0), ChVector<>(0.073966, 0.01, 0.02), "pad2_Link"}
};

// JointData scorbot_joints[] = {
//     {"base_joint", "link0", "link1", ChVector<>(0, 0.2274, 0), ChVector<>(0,0,0), ChVector<>(0,0,1)},
//     {"shoulder_joint", "link1", "link2", ChVector<>(0.0255, 0.13, 0), ChVector<>(-CH_C_PI_2, -CH_C_PI_2, 0), ChVector<>(0,0,1)},
//     {"elbow_joint", "link2", "link3", ChVector<>(0.2225, 0, 0), ChVector<>(0,0,0), ChVector<>(0,0,1)},
//     {"pitch_joint", "link3", "link4", ChVector<>(0.2225, 0, 0), ChVector<>(1.570796327,0,0), ChVector<>(0,1,0)},
//     {"roll_joint", "link4", "link5", ChVector<>(0,0,0), ChVector<>(0, 0, -1.570796327), ChVector<>(0,0,1)},
//     {"pad1_joint", "link5", "pad1_link", ChVector<>(0, -0.0905, -0.0545), ChVector<>(CH_C_PI_2, -CH_C_PI_2, -0.74758), ChVector<>(0, 0, 1) },
//     {"pad2_joint", "link5", "pad2_link", ChVector<>(0, -0.0905, 0.0545), ChVector<>(-CH_C_PI_2, CH_C_PI_2, -0.74758), ChVector<>(0, 0, -1) }
// };

JointData scorbot_joints[] = {
    {"base_joint", "link0", "link1", ChVector<>(0.15, 0.2274, 0.15), ChVector<>(0,0,0), ChVector<>(0,0,1), -CH_C_PI, CH_C_PI},
    {"shoulder_joint", "link1", "link2", ChVector<>(0, 0.2, 0.0255), ChVector<>(-CH_C_PI_2, -CH_C_PI_2, 0), ChVector<>(0,0,1), -0.610865, 2.268928},
    {"elbow_joint", "link2", "link3", ChVector<>(0, 0, 0.2225), ChVector<>(0,0,0), ChVector<>(0,0,1), -2.268928, 2.268928},
    {"pitch_joint", "link3", "link4", ChVector<>(0.2225, 0, 0), ChVector<>(-1.570796327,0, 0), ChVector<>(0,1,0), -2.268928, 2.268928},
    {"roll_joint", "link4", "link5", ChVector<>(0,0,0), ChVector<>(CH_C_PI, 0, 0), ChVector<>(0,0,1), -CH_C_PI, CH_C_PI},
    {"pad1_joint", "link5", "pad1_link", ChVector<>(0, -0.0905, -0.0545), ChVector<>(CH_C_PI_2, 0.74758, CH_C_PI_2), ChVector<>(0, 0, 1), 0, CH_C_PI_2},
    {"pad2_joint", "link5", "pad2_link", ChVector<>(0, -0.0905,  0.0545), ChVector<>(CH_C_PI_2, -0.74758, CH_C_PI_2), ChVector<>(0, 0, -1), 0, CH_C_PI_2 }

};



// given link name, return LinkData, note that this is for child only, parent is based on previous result!
LinkData LookupLinkData(std::string linkname){
    LinkData tmp_link;
    for (int i = 0; i < numLinks; i++){
        tmp_link = scorbot_links[i];
        if (tmp_link.name.compare(linkname) == 0){
            break;
        }
    }
    return tmp_link;
}

// add visual assets to the body
void AddVisualizationShape(std::shared_ptr<ChBodyAuxRef> body_ptr,
                           VisualizationType type){

    if (type == VisualizationType::BOX){
        auto box_shape = chrono_types::make_shared<ChBoxShape>();
        box_shape->GetBoxGeometry().SetLengths(LookupLinkData(body_ptr->GetNameString()).dim);
        box_shape->SetColor(ChColor(ChRandom(), ChRandom(), ChRandom()));
        body_ptr->AddVisualShape(box_shape);

        // std::cout << "add shape of dimenstion" << LookupLinkData(body_ptr->GetNameString()).dim * 0.2 << " at location " <<  body_ptr->GetFrame_COG_to_abs() << std::endl;
    }

    if (type == VisualizationType::MESH){
        std::string partname = body_ptr->GetNameString();
        std::string meshname = LookupLinkData(partname).meshname;
        std::string vis_mesh_file = "/home/luning/CollaborateProjects/Scorbot/data/obj/" + meshname + ".obj";



        auto trimesh_vis = geometry::ChTriangleMeshConnected::CreateFromWavefrontFile(vis_mesh_file, true, true);
        
        if (body_ptr->GetNameString() == "link2"){
            trimesh_vis->Transform(VNULL, Q_from_AngX(-CH_C_PI_2) * Q_from_AngY(-CH_C_PI_2));
        }

        else if (body_ptr->GetNameString() == "link3"){
            trimesh_vis->Transform(VNULL, Q_from_AngZ(CH_C_PI_2));
        }

        else {
            trimesh_vis->Transform(VNULL, Q_from_AngX(-CH_C_PI_2));
        }


        auto trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
        trimesh_shape->SetMesh(trimesh_vis);
        trimesh_shape->SetMutable(false);
        trimesh_shape->SetColor(ChColor(ChRandom(), ChRandom(), ChRandom()));
        // trimesh_shape->SetColor(ChColor(1,1,1));

        body_ptr->AddVisualShape(trimesh_shape);
        

    }

    if (type == VisualizationType::CYLINDER){
        auto cyl2 = chrono_types::make_shared<ChCylinderShape>();
        cyl2->GetCylinderGeometry().p1 = ChVector<>(0, 0, 0);
        cyl2->GetCylinderGeometry().p2 = ChVector<>(0.24, 0, 0);
        cyl2->GetCylinderGeometry().rad = 0.03;
        cyl2->SetColor(ChColor(0.2, 0.8, 0.3f));
        body_ptr->AddVisualShape(cyl2);
    }
}


// auxillary frame, not at the center of gravity
// note that the parent info is based on its parents, 
std::shared_ptr<ChBodyAuxRef> createChildAuxRef(ChQuaternion<> parent_quat,
                                                ChVector<> parent_location,
                                                int i // joint id (start from 0)
                                                ){
    auto child = chrono_types::make_shared<ChBodyAuxRef>();
    ChQuaternion<> joint_quat = parent_quat * rpy2quat(scorbot_joints[i].origin_rpy); // this need to be changed ...

    // perform transformation using quaternion to get global location of the joint
    ChVector<double> joint_location_global = parent_location + parent_quat.Rotate(scorbot_joints[i].origin_xyz);

    // std::cout << "parent location: aux wrt global " << parent_location << std::endl;

    child->SetFrame_REF_to_abs(ChFrame<>(joint_location_global, joint_quat));  
    // child info can be read directly from urdf! 
    LinkData child_data = LookupLinkData(scorbot_joints[i].link_child);
    child->SetFrame_COG_to_REF(ChFrame<>(child_data.origin_xyz, rpy2quat(child_data.origin_rpy)));
    return child;
} 



int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";
    ChSystemNSC sys;
    std::unordered_map<std::string,  std::shared_ptr<ChBodyAuxRef>> name_link_map; // map that store link and the pointer to the body once created in Chrono (in absolute coordinate frames!)


    // start with prototype
    bool useMesh = false;
    // ChVector<float> gravity(0, 0, 9.81);
    ChVector<float> gravity(0, 0, 0);
    sys.Set_G_acc(gravity);
    double step_size = 1e-3; // 1e-3 in world.skel which one?

    double mass_base = 3.5;
    double mass_link1 = 2.5;
    double mass_link2 = 3;

    ChVector<double> dim_base_link(0.24, 0.24, 0.2274);
    ChVector<double> dim_link1(0.205, 0.26, 0.17);
    ChVector<double> dim_link2(0.28749, 0.069982, 0.169);


    auto link_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    auto link_mat_vis = chrono_types::make_shared<ChVisualMaterial>(*ChVisualMaterial::Default());

    std::shared_ptr<collision::ChCollisionModel> collision_model = chrono_types::make_shared<collision::ChCollisionModelBullet>();

    // add base link, start from origin, add name to aux ref for easy lookup?
    auto base_link = chrono_types::make_shared<ChBodyAuxRef>();
    LinkData base_link_data = scorbot_links[0];
    base_link->SetNameString(base_link_data.name);
    base_link->SetFrame_REF_to_abs(ChFrame<>(base_link_data.origin_xyz, rpy2quat(base_link_data.origin_rpy)));
    base_link->SetFrame_COG_to_REF(ChFrame<>(ChVector<>(0, 0.1137, 0), ChQuaternion<>(1,0,0,0)));  

    base_link->SetBodyFixed(true);
    base_link->SetCollide(false);
    base_link->SetMass(mass_base);
    base_link->SetInertiaXX(ChVector<>(0.2, 0.2, 0.2)); // TODO: inertia? w.r.t. which axes?
    
    AddVisualizationShape(base_link, VisualizationType::MESH);
    sys.Add(base_link);
    std::cout << base_link->GetNameString() << " Absolute position of COG: " << base_link->GetFrame_COG_to_abs() << std::endl;
    std::cout << base_link->GetNameString() << " Absolute position of AUX: " << base_link->GetFrame_REF_to_abs() << std::endl;

    // insert this to my map
    name_link_map[base_link_data.name] = base_link;

    for (int i = 0; i < numJoints; i++){
        // find body pointer 
        std::string parent_name = scorbot_joints[i].link_parent;
        std::shared_ptr<ChBodyAuxRef> parent_body = name_link_map.find(parent_name)->second; // parent body

        std::shared_ptr<ChBodyAuxRef> child_body  = createChildAuxRef(parent_body->GetFrame_REF_to_abs().GetRot(), parent_body->GetFrame_REF_to_abs().GetPos(), i);
        child_body->SetNameString(scorbot_joints[i].link_child);

        // std::cout << "child name: " << child_body->GetNameString() << " absolute position of COG: " << child_body->GetFrame_COG_to_abs() << std::endl;
        child_body->SetBodyFixed(false);
        child_body->SetMass(mass_link1);  // TODO mass
        child_body->SetCollide(false);
        child_body->SetInertiaXX(ChVector<>(0.2, 0.2, 0.2)); // TODO: inertia? w.r.t. which axes?
    
        AddVisualizationShape(child_body, VisualizationType::MESH);
        sys.Add(child_body);
        
        std::cout << child_body->GetNameString() << " Absolute position of COG: " << child_body->GetFrame_COG_to_abs() << std::endl;
        std::cout << child_body->GetNameString() << " Absolute position of AUX: " << child_body->GetFrame_REF_to_abs() << std::endl;



        // insert this to my map
        name_link_map[child_body->GetNameString()] = child_body;

        if (i == 0 || i == 1 || i == 2){
            // revolute joint motor between base and shoulder
            auto rev_base_shoulder = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
            rev_base_shoulder->Initialize(parent_body, child_body, ChFrame<>(child_body->GetFrame_REF_to_abs().GetPos(), child_body->GetFrame_REF_to_abs().GetRot() * Q_from_AngX(CH_C_PI_2) ));
            auto mwspeed =
                chrono_types::make_shared<ChFunction_Const>(CH_C_PI_4);  // constant angular speed, in [rad/s], 1PI/s =180°/s

            rev_base_shoulder->SetSpeedFunction(mwspeed);
            sys.AddLink(rev_base_shoulder);

        }

    }

    // revolute joint motor between base and shoulder
    // auto rev_base_shoulder = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
    // rev_base_shoulder->Initialize(base_link, link1, ChFrame<>(pos_base_joint, ChQuaternion<>(1, 0, 0, 0)));
    // auto mwspeed =
    //     chrono_types::make_shared<ChFunction_Const>(CH_C_PI_4);  // constant angular speed, in [rad/s], 1PI/s =180°/s

    // rev_base_shoulder->SetSpeedFunction(mwspeed);
    // sys.AddLink(rev_base_shoulder);

    // Add limits to the Z rotation of the revolute joint
    // double min_angle = 0;
    // double max_angle = 0.75 * CH_C_PI;
    // rev->GetLimit_Rz().SetActive(true);
    // rev->GetLimit_Rz().SetMin(min_angle);
    // rev->GetLimit_Rz().SetMax(max_angle);
    // Initialize the joint specifying a coordinate sys (expressed in the absolute frame).

    /////////////////
    //Visualization//
    /////////////////


    // Create the Irrlicht visualization system
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->AttachSystem(&sys);
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("NSC collision demo");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddCamera(ChVector<>(1, 1.5, 0.5));
    vis->AddTypicalLights();

    int frame = 0;

    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();
        // vis->EnableBodyFrameDrawing(true);
        tools::drawSegment(vis.get(), ChVector<>(0,0,0), ChVector<>(10,0,0), ChColor(0.6f, 0, 0), true);
        tools::drawSegment(vis.get(), ChVector<>(0,0,0), ChVector<>(0,10,0), ChColor(0, 0.6f, 0), true);
        tools::drawSegment(vis.get(), ChVector<>(0,0,0), ChVector<>(0,0,10), ChColor(0, 0, 0.6f), true);

        vis->EndScene();

        sys.DoStepDynamics(step_size);


    }


    return 0;
}
