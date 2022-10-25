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
// todo: motor at revolute joint
// gaits
// record contact force (Callback)
// use 13cm ones
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


// Convert a triplet (roll-pitch-yaw) to a quaternion
ChQuaternion<> rpy2quat(const ChVector<>& rpy) {
    return Q_from_AngZ(rpy.z()) * Q_from_AngY(rpy.y()) * Q_from_AngX(rpy.x());
}


struct LinkData{
    std::string name;
    ChVector<> origin_xyz;
    ChVector<> origin_rpy;
    
};


struct JointData{
    std::string name;
    std::string link_parent;
    std::string link_child;
    ChVector<> origin_xyz;
    ChVector<> origin_rpy;
    ChVector<> axis;
};

JointData scorbot_joints[] = {
    {"base_joint", "base_link", "link1", ChVector<>(0,0,0.2274), ChVector<>(0,0,0), ChVector<>(0,0,1)},
    {"shoulder_joint", "link1", "link2", ChVector<>(0.0255, 0, 0.13), ChVector<>(-CH_C_PI_2, 0, -CH_C_PI_2), ChVector<>(0,0,1)},
    {"elbow_joint", "link2", "link3", ChVector<>(0.2225, 0, 0), ChVector<>(0,0,0), ChVector<>(0,0,1)}
};



collision::ChCollisionSystemType collision_type = collision::ChCollisionSystemType::BULLET;


// auxillary frame, not at the center of gravity!!
std::shared_ptr<ChBodyAuxRef> createChildAuxRef(ChQuaternion<> parent_frame,
                                           ChVector<> parent_location,
                                           int i // joint id (start from 0)
                                           ){
    auto child = chrono_types::make_shared<ChBodyAuxRef>();
    ChMatrix33<> parentRot(parent_frame);
    ChMatrix33<> jointRot(rpy2quat(scorbot_joints[i].origin_rpy)); // this need to be changed ...

    ChVector<double> joint_location_global = parent_location + parentRot * scorbot_joints[i].origin_xyz;
    

    child->SetFrame_REF_to_abs(ChFrame<>(joint_location_global, rpy2quat(scorbot_joints[i].origin_rpy)));
  
    child->SetFrame_COG_to_REF(ChFrame<>(ChVector<>(0.11, 0, 0), ChQuaternion<>(1, 0, 0, 0)));

    return child;
} 


int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";
    ChSystemNSC sys;

    // start with prototype
    bool useMesh = false;
    // ChVector<float> gravity(0, 0, 9.81);
    ChVector<float> gravity(0, 0, -9.81);
    sys.Set_G_acc(gravity);
    double step_size = 1e-3; // 1e-3 in world.skel which one?

    double mass_base = 3.5;
    double mass_link1 = 2.5;
    double mass_link2 = 3;

    ChVector<double> dim_base_link(0.24, 0.24, 0.2274);
    ChVector<double> dim_link1(0.205, 0.26, 0.17);
    ChVector<double> dim_link2(0.28749, 0.069982, 0.169);

    // ChVector<double> dim_base_link(0.024, 0.024, 0.02274);
    // ChVector<double> dim_link1(0.0205, 0.026, 0.017);
    // ChVector<double> dim_link2(0.028749, 0.0069982, 0.0169);


    // ChVector<double> pos_base_link(0.27, 0.0, 0.469);
    ChVector<double> pos_base_link(0.27, 0.0, 0.469);
    ChVector<double> pos_base_joint = pos_base_link + ChVector<double>(0, 0, 0.2274);
    ChVector<double> pos_link1 = pos_base_joint + ChVector<double>(0, 0, 0.075);




    // ChVector<double> pos_link1(0.27, 0.0, 0.469);

    auto link_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    auto link_mat_vis = chrono_types::make_shared<ChVisualMaterial>(*ChVisualMaterial::Default());

    std::shared_ptr<collision::ChCollisionModel> collision_model = chrono_types::make_shared<collision::ChCollisionModelBullet>();
    auto base_link = chrono_types::make_shared<ChBodyEasyBox>(dim_base_link.x(), 
                                                              dim_base_link.y(), 
                                                              dim_base_link.z(),  // x,y,z size
                                                              100,        // density
                                                              true,       // visualization?
                                                              true,
                                                              link_mat,
                                                              collision_model);     // collision?
    base_link->SetPos(pos_base_link);
    base_link->SetBodyFixed(true);
    base_link->SetMass(mass_base);

    
    if (useMesh) {
        std::string vis_mesh_file = "/home/luning/CollaborateProjects/scorbot/data/obj/base_Link.obj";
        auto trimesh_vis = geometry::ChTriangleMeshConnected::CreateFromWavefrontFile(vis_mesh_file, true, true);
            // trimesh_vis->Transform(m_mesh_xform.GetPos(), m_mesh_xform.GetA());  // translate/rotate/scale mesh
            // trimesh_vis->RepairDuplicateVertexes(1e-9);                          // if meshes are not watertight

        auto trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
        trimesh_shape->SetMesh(trimesh_vis);
        trimesh_shape->SetName("base_Link");
        trimesh_shape->SetMutable(false);
        // ChFrame<> transform(pos_base_link, QUNIT);    
        // ChFrame<> transform(ChVector<double>(0, 0, 0), QUNIT);    

        // trimesh_vis->Transform(transform.GetPos(), transform.GetA());  // translate/rotate/scale mesh
        trimesh_shape->SetColor(ChColor(0.6f, 0, 0));

        base_link->AddVisualShape(trimesh_shape);
    }

    sys.AddBody(base_link);



    std::shared_ptr<collision::ChCollisionModel> collision_model2 = chrono_types::make_shared<collision::ChCollisionModelBullet>();
    auto link1 = chrono_types::make_shared<ChBodyEasyBox>(dim_link1.x(), 
                                                              dim_link1.y(), 
                                                              dim_link1.z(),  // x,y,z size
                                                              100,        // density
                                                              true,       // visualization?
                                                              true,
                                                              link_mat,
                                                              collision_model2);     // collision?
    link1->SetPos(pos_link1);
    link1->SetBodyFixed(true);
    link1->SetMass(mass_link1);

    if (useMesh){
        std::string shoulder_mesh_file = "/home/luning/CollaborateProjects/scorbot/data/obj/shoulder_Link.obj";
        auto shoulder_vis = geometry::ChTriangleMeshConnected::CreateFromWavefrontFile(shoulder_mesh_file, true, true);
            // trimesh_vis->Transform(m_mesh_xform.GetPos(), m_mesh_xform.GetA());  // translate/rotate/scale mesh
            // trimesh_vis->RepairDuplicateVertexes(1e-9);                          // if meshes are not watertight

        auto shoulder_trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
        shoulder_trimesh_shape->SetMesh(shoulder_vis);
        shoulder_trimesh_shape->SetName("shoulder_Link");
        shoulder_trimesh_shape->SetMutable(false);
        // trimesh_vis->Transform(transform_link1.GetPos(), transform_link1.GetA());  // translate/rotate/scale mesh
        base_link->AddVisualShape(shoulder_trimesh_shape);

    }



    sys.AddBody(link1);

    // link2
    ChVector<> shoulder_joint_eu(-CH_C_PI_2, -CH_C_PI_2, 0);
    ChQuaternion<> shoulder_joint_frame = Q_from_Euler123(shoulder_joint_eu);
    std::shared_ptr<ChBodyAuxRef> link2 = createChildAuxRef(link1->GetRot(), link1->GetPos(), 1);

    link2->SetBodyFixed(true);
    link2->SetCollide(false);
    link2->SetMass(mass_link2);
    link2->SetInertiaXX(ChVector<>(0.2, 0.2, 0.2)); // inertia? w.r.t. which axes?
    
    auto cyl = chrono_types::make_shared<ChCylinderShape>();
    cyl->GetCylinderGeometry().p1 = ChVector<>(0, 0, 0);
    cyl->GetCylinderGeometry().p2 = ChVector<>(0.22, 0, 0);
    cyl->GetCylinderGeometry().rad = 0.05;
    cyl->SetColor(ChColor(0, 0, 0.6f));
    link2->AddVisualShape(cyl);
    
    sys.Add(link2);
    std::cout << "after adding body\n: body frame: \n" << link2->GetRot() << std::endl;

    // link3 -- pitch
    ChVector<> elbow_joint_eu(0,0,0);
    ChQuaternion<> elbow_joint_frame = Q_from_Euler123(elbow_joint_eu);
    ChVector<> elbow_joint_location(0.2225, 0, 0);
    std::shared_ptr<ChBodyAuxRef> link3 = createChildAuxRef(link2->GetFrame_REF_to_abs().GetRot(), link2->GetFrame_REF_to_abs().GetPos(), 2);


    auto cyl2 = chrono_types::make_shared<ChCylinderShape>();
    cyl2->GetCylinderGeometry().p1 = ChVector<>(0, 0, 0);
    cyl2->GetCylinderGeometry().p2 = ChVector<>(0.24, 0, 0);
    cyl2->GetCylinderGeometry().rad = 0.03;
    cyl2->SetColor(ChColor(0.2, 0.8, 0.3f));
    link3->AddVisualShape(cyl2);
    link3->SetBodyFixed(true);
    sys.Add(link3);



    // revolute joint motor between base and shoulder
    auto rev_base_shoulder = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
    rev_base_shoulder->Initialize(base_link, link1, ChFrame<>(pos_base_joint, ChQuaternion<>(1, 0, 0, 0)));
    auto mwspeed =
        chrono_types::make_shared<ChFunction_Const>(CH_C_PI_4);  // constant angular speed, in [rad/s], 1PI/s =180°/s

    rev_base_shoulder->SetSpeedFunction(mwspeed);
    sys.AddLink(rev_base_shoulder);

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
    vis->AddCamera(ChVector<>(1, 0, 1.5));
    vis->AddTypicalLights();

    int frame = 0;

    while (vis->Run()) {
        vis->BeginScene();
        vis->EnableBodyFrameDrawing(true);
        vis->Render();
        tools::drawSegment(vis.get(), ChVector<>(0,0,0), ChVector<>(10,0,0), ChColor(0.6f, 0, 0), true);
        tools::drawSegment(vis.get(), ChVector<>(0,0,0), ChVector<>(0,10,0), ChColor(0, 0.6f, 0), true);
        tools::drawSegment(vis.get(), ChVector<>(0,0,0), ChVector<>(0,0,10), ChColor(0, 0, 0.6f), true);

        vis->EndScene();

        sys.DoStepDynamics(step_size);


    }


    return 0;
}
