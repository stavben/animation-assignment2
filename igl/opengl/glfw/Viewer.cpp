// This file is part of libigl, a simple c++ geometry processing library.
//
// Copyright (C) 2014 Daniele Panozzo <daniele.panozzo@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.

#include "Viewer.h"

//#include <chrono>
#include <thread>

#include <Eigen/LU>


#include <cmath>
#include <cstdio>
#include <sstream>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <limits>
#include <cassert>

#include <igl/project.h>
//#include <igl/get_seconds.h>
#include <igl/readOBJ.h>
#include <igl/readOFF.h>
#include <igl/adjacency_list.h>
#include <igl/writeOBJ.h>
#include <igl/writeOFF.h>
#include <igl/massmatrix.h>
#include <igl/file_dialog_open.h>
#include <igl/file_dialog_save.h>
#include <igl/quat_mult.h>
#include <igl/axis_angle_to_quat.h>
#include <igl/trackball.h>
#include <igl/two_axis_valuator_fixed_up.h>
#include <igl/snap_to_canonical_view_quat.h>
#include <igl/unproject.h>
#include <igl/serialize.h>

// Internal global variables used for glfw event handling
//static igl::opengl::glfw::Viewer * __viewer;
static double highdpi = 1;
static double scroll_x = 0;
static double scroll_y = 0;


namespace igl
{
namespace opengl
{
namespace glfw
{

  void Viewer::Init(const std::string config)
  {
	  

  }

 /* void Viewer::SetQueue()
  {
  }*/

  IGL_INLINE Viewer::Viewer():
    data_list(1),
    selected_data_index(0),
    next_data_id(1),
	isPicked(false),
	isActive(false)
    //isActive(true)
  {
    data_list.front().id = 0;

  

    // Temporary variables initialization
   // down = false;
  //  hack_never_moved = true;
    scroll_position = 0.0f;

    // Per face
    data().set_face_based(false);

    
#ifndef IGL_VIEWER_VIEWER_QUIET
    const std::string usage(R"(igl::opengl::glfw::Viewer usage:
  [drag]  Rotate scene
  A,a     Toggle animation (tight draw loop)
  F,f     Toggle face based
  I,i     Toggle invert normals
  L,l     Toggle wireframe
  O,o     Toggle orthographic/perspective projection
  T,t     Toggle filled faces
  [,]     Toggle between cameras
  1,2     Toggle between models
  ;       Toggle vertex labels
  :       Toggle face labels)"
);
    std::cout<<usage<<std::endl;
#endif
  }

  IGL_INLINE Viewer::~Viewer()
  {
  }

  IGL_INLINE bool Viewer::load_mesh_from_file(
      const std::string & mesh_file_name_string)
  {

    // Create new data slot and set to selected
    if(!(data().F.rows() == 0  && data().V.rows() == 0))
    {
      append_mesh();
    }
    data().clear();

    size_t last_dot = mesh_file_name_string.rfind('.');
    if (last_dot == std::string::npos)
    {
      std::cerr<<"Error: No file extension found in "<<
        mesh_file_name_string<<std::endl;
      return false;
    }

    std::string extension = mesh_file_name_string.substr(last_dot+1);

    if (extension == "off" || extension =="OFF")
    {
      Eigen::MatrixXd V;
      Eigen::MatrixXi F;
      if (!igl::readOFF(mesh_file_name_string, V, F))
        return false;
      data().set_mesh(V,F);
      //-------------------------------------------------------------assignment2
     //data().tree.init(V, F);
    }
    else if (extension == "obj" || extension =="OBJ")
    {
      Eigen::MatrixXd corner_normals;
      Eigen::MatrixXi fNormIndices;

      Eigen::MatrixXd UV_V;
      Eigen::MatrixXi UV_F;
      Eigen::MatrixXd V;
      Eigen::MatrixXi F;

      if (!(
            igl::readOBJ(
              mesh_file_name_string,
              V, UV_V, corner_normals, F, UV_F, fNormIndices)))
      {
        return false;
      }

      data().set_mesh(V,F);
      //----------------------------------------------------------assignment2
      //data().tree.init(V, F);
      if (UV_V.rows() > 0)
      {
          data().set_uv(UV_V, UV_F);
      }

    }
    else
    {
      // unrecognized file type
      printf("Error: %s is not a recognized file type.\n",extension.c_str());
      return false;
    }

    data().compute_normals();
    data().uniform_colors(Eigen::Vector3d(51.0/255.0,43.0/255.0,33.3/255.0),
                   Eigen::Vector3d(255.0/255.0,228.0/255.0,58.0/255.0),
                   Eigen::Vector3d(255.0/255.0,235.0/255.0,80.0/255.0));

    // Alec: why?
    if (data().V_uv.rows() == 0)
    {
      data().grid_texture();
    }
    

    //for (unsigned int i = 0; i<plugins.size(); ++i)
    //  if (plugins[i]->post_load())
    //    return true;

    return true;
  }

  IGL_INLINE bool Viewer::save_mesh_to_file(
      const std::string & mesh_file_name_string)
  {
    // first try to load it with a plugin
    //for (unsigned int i = 0; i<plugins.size(); ++i)
    //  if (plugins[i]->save(mesh_file_name_string))
    //    return true;

    size_t last_dot = mesh_file_name_string.rfind('.');
    if (last_dot == std::string::npos)
    {
      // No file type determined
      std::cerr<<"Error: No file extension found in "<<
        mesh_file_name_string<<std::endl;
      return false;
    }
    std::string extension = mesh_file_name_string.substr(last_dot+1);
    if (extension == "off" || extension =="OFF")
    {
      return igl::writeOFF(
        mesh_file_name_string,data().V,data().F);
    }
    else if (extension == "obj" || extension =="OBJ")
    {
      Eigen::MatrixXd corner_normals;
      Eigen::MatrixXi fNormIndices;

      Eigen::MatrixXd UV_V;
      Eigen::MatrixXi UV_F;

      return igl::writeOBJ(mesh_file_name_string,
          data().V,
          data().F,
          corner_normals, fNormIndices, UV_V, UV_F);
    }
    else
    {
      // unrecognized file type
      printf("Error: %s is not a recognized file type.\n",extension.c_str());
      return false;
    }
    return true;
  }
 
  IGL_INLINE bool Viewer::load_scene()
  {
    std::string fname = igl::file_dialog_open();
    if(fname.length() == 0)
      return false;
    return load_scene(fname);
  }

  IGL_INLINE bool Viewer::load_scene(std::string fname)
  {
   // igl::deserialize(core(),"Core",fname.c_str());
    igl::deserialize(data(),"Data",fname.c_str());
    return true;
  }

  IGL_INLINE bool Viewer::save_scene()
  {
    std::string fname = igl::file_dialog_save();
    if (fname.length() == 0)
      return false;
    return save_scene(fname);
  }

  IGL_INLINE bool Viewer::save_scene(std::string fname)
  {
    //igl::serialize(core(),"Core",fname.c_str(),true);
    igl::serialize(data(),"Data",fname.c_str());

    return true;
  }

  IGL_INLINE void Viewer::open_dialog_load_mesh()
  {
    std::string fname = igl::file_dialog_open();

    if (fname.length() == 0)
      return;
    
    this->load_mesh_from_file(fname.c_str());
  }

  IGL_INLINE void Viewer::open_dialog_save_mesh()
  {
    std::string fname = igl::file_dialog_save();

    if(fname.length() == 0)
      return;

    this->save_mesh_to_file(fname.c_str());
  }

  IGL_INLINE ViewerData& Viewer::data(int mesh_id /*= -1*/)
  {
    assert(!data_list.empty() && "data_list should never be empty");
    int index;
    if (mesh_id == -1)
      index = selected_data_index;
    else
      index = mesh_index(mesh_id);

    assert((index >= 0 && index < data_list.size()) &&
      "selected_data_index or mesh_id should be in bounds");
    return data_list[index];
  }

  IGL_INLINE const ViewerData& Viewer::data(int mesh_id /*= -1*/) const
  {
    assert(!data_list.empty() && "data_list should never be empty");
    int index;
    if (mesh_id == -1)
      index = selected_data_index;
    else
      index = mesh_index(mesh_id);

    assert((index >= 0 && index < data_list.size()) &&
      "selected_data_index or mesh_id should be in bounds");
    return data_list[index];
  }

  IGL_INLINE int Viewer::append_mesh(bool visible /*= true*/)
  {
    assert(data_list.size() >= 1);

    data_list.emplace_back();
    selected_data_index = data_list.size()-1;
    data_list.back().id = next_data_id++;
    //if (visible)
    //    for (int i = 0; i < core_list.size(); i++)
    //        data_list.back().set_visible(true, core_list[i].id);
    //else
    //    data_list.back().is_visible = 0;
    return data_list.back().id;
  }

  IGL_INLINE bool Viewer::erase_mesh(const size_t index)
  {
    assert((index >= 0 && index < data_list.size()) && "index should be in bounds");
    assert(data_list.size() >= 1);
    if(data_list.size() == 1)
    {
      // Cannot remove last mesh
      return false;
    }
    data_list[index].meshgl.free();
    data_list.erase(data_list.begin() + index);
    if(selected_data_index >= index && selected_data_index > 0)
    {
      selected_data_index--;
    }

    return true;
  }

  IGL_INLINE size_t Viewer::mesh_index(const int id) const {
    for (size_t i = 0; i < data_list.size(); ++i)
    {
      if (data_list[i].id == id)
        return i;
    }
    return 0;
  }

  Eigen::Matrix4d Viewer::CalcParentsTrans(int indx) 
  {
	  Eigen::Matrix4d prevTrans = Eigen::Matrix4d::Identity();

	  for (int i = indx; parents[i] >= 0; i = parents[i])
	  {
		  //std::cout << "parent matrix:\n" << scn->data_list[scn->parents[i]].MakeTrans() << std::endl;
		  prevTrans = data_list[parents[i]].MakeTransd() * prevTrans;
	  } 

	  return prevTrans;
  }

  //-------------------------------------------------------------------------------------------assignment2
  /*IGL_INLINE bool Viewer::isIntersectBox(Eigen::AlignedBox3d& box0, Eigen::AlignedBox3d& box1,
      Eigen::Matrix4d& model0, Eigen::Matrix4d& model1,
      Eigen::Matrix3d& Rot0, Eigen::Matrix3d& Rot1) {
      Eigen::MatrixXd A = data_list[0].GetRotation(), B = data_list[1].GetRotation();
      Eigen::MatrixXd C = A.transpose() * B;

      Eigen::RowVectorXd C0asd = data_list[0].tree.m_box.center(), C1asd = data_list[1].tree.m_box.center();
      Eigen::Vector4d Aright(data_list[0].MakeTransd()(0, 3), data_list[0].MakeTransd()(1, 3), data_list[0].MakeTransd()(2, 3), 1);
      Eigen::Vector4d Bright(data_list[1].MakeTransd()(0, 3), data_list[1].MakeTransd()(1, 3), data_list[1].MakeTransd()(2, 3), 1);
      Eigen::Vector4d C0(C0asd(0), C0asd(1), C0asd(2), 1);
      Eigen::Vector4d C1(C1asd(0), C1asd(1), C1asd(2), 1);
      Eigen::Vector4d D = (data_list[1].MakeTransd() * C1) - (data_list[0].MakeTransd() * C0);
      double a[] = { data_list[0].tree.m_box.sizes().x() / 2, data_list[0].tree.m_box.sizes().y() / 2, data_list[0].tree.m_box.sizes().z() / 2 };
      double b[] = { data_list[1].tree.m_box.sizes().x() / 2, data_list[1].tree.m_box.sizes().y() / 2, data_list[1].tree.m_box.sizes().z() / 2 };
      printf("a (%f, %f, %f )\n", a[0], a[1], a[2]);
      printf("b (%f, %f, %f )\n", b[0], b[1], b[2]);

      std::cout << "D:\n" << D[0] << std::endl;

      double R, R0, R1; // For readability by the pdf in lecture
      for (int i = 0; i <= 2; i++) {
          // tests of 1,2,3
          R0 = a[i];
          R1 = b[0] * abs(C(i, 0)) + b[1] * abs(C(i, 1)) + b[2] * abs(C(i, 2)); //abs = absolute value
          R = abs(A.col(i).transpose() * D);
          if (R > (R0 + R1)) {
              return true;
          }

          std::cout<< "\nim here\n ";
          // Tests of 4,5,6
          R0 = a[0] * abs(C(0, i)) + a[1] * abs(C(1, i)) + a[2] * abs(C(2, i));
          R1 = b[i];
          R = abs(B.col(i).dot(D));
          if (R > (R0 + R1)) {
              return true;
          }

          // Tests 7,8,9,10,11,12,13,14,15
          for (int j = 0; j <= 2; j++) {
              R0 = a[(i == 0 ? 1 : 0)] * abs(C((i == 2 ? 1 : 2), j)) + a[(i == 2 ? 1 : 2)] * abs(C((i == 0 ? 1 : 0), j));
              R1 = b[(j == 0 ? 1 : 0)] * abs(C(i, (j == 2 ? 1 : 2))) + b[(j == 2 ? 1 : 2)] * abs(C(i, (j == 0 ? 1 : 0)));
              R = abs(C(((i + 1) % 3), j) * A.col(((i + 2) % 3)).dot(D) - C(((i + 2) % 3), j) * A.col(((i + 1) % 3)).dot(D));
              if (R > (R0 + R1)) {
                  return true;
              }
          }

      }






      */


  IGL_INLINE bool Viewer::isIntersectBox(Eigen::AlignedBox3d& box0, Eigen::AlignedBox3d& box1,
      Eigen::Matrix4d& model0, Eigen::Matrix4d& model1,
      Eigen::Matrix3d& Rot0, Eigen::Matrix3d& Rot1) {
      Eigen::Vector3d a = box0.sizes() / 2;
      Eigen::Vector3d b = box1.sizes() /2; 
      Eigen::Vector4d temp1;                         // Help to convert vector of 4 to 3
      Eigen::MatrixXd A = data_list[0].GetRotation(), B = data_list[1].GetRotation();
      temp1 << box0.center(), 1; //center of first box
      Eigen::Vector3d C0 = (model0 * temp1).block<3, 1>(0, 0);
      temp1 << box1.center(), 1; //center of second box
      Eigen::Vector3d C1 = (model1 * temp1).block<3, 1>(0, 0);
      Eigen::Vector3d D = C1 - C0;
      Eigen::Matrix3d C = A.transpose() * B;
      
      double R, R0, R1; // For readability by the pdf in lecture
      for (int i = 0; i <= 2; i++) {
          // tests of 1,2,3
          R0 = a(i);
          R1 = b(0) * abs(C(i, 0)) + b(1) * abs(C(i, 1)) + b(2) * abs(C(i, 2)); //abs = absolute value
          R = abs(A.col(i).dot(D));
          if (R > (R0 + R1)) {
              return true;  
          }
          // Tests of 4,5,6
          R0 = a(0) * abs(C(0, i)) + a(1) * abs(C(1, i)) + a(2) * abs(C(2, i));
          R1 = b(i);
          R = abs(B.col(i).dot(D));
          if (R > (R0 + R1)) {
              return true;
          }
          // Tests 7,8,9,10,11,12,13,14,15
          for (int j = 0; j <= 2; j++) {
              R0 = a((i == 0 ? 1 : 0)) * abs(C((i == 2 ? 1 : 2), j)) + a((i == 2 ? 1 : 2)) * abs(C((i == 0 ? 1 : 0), j));
              R1 = b((j == 0 ? 1 : 0)) * abs(C(i, (j == 2 ? 1 : 2))) + b((j == 2 ? 1 : 2)) * abs(C(i, (j == 0 ? 1 : 0)));
              R = abs(C(((i + 1) % 3), j) * A.col(((i + 2) % 3)).dot(D) - C(((i + 2) % 3), j) * A.col(((i + 1) % 3)).dot(D));
              if (R > (R0 + R1)) {
                  return true;
              }
          }   
      }
      return false; 
  }
  
 


  IGL_INLINE bool Viewer::isIntersection(igl::AABB<Eigen::MatrixXd, 3>* tree0, igl::AABB<Eigen::MatrixXd, 3>* tree1,
      Eigen::Matrix4d& model0, Eigen::Matrix4d& model1,
      Eigen::Matrix3d& Rot0, Eigen::Matrix3d& Rot1) {
      
      if (tree0->is_leaf() && !tree1->is_leaf()) {
          return isIntersection(tree0, tree1->m_left, model0, model1, Rot0, Rot1) || isIntersection(tree0, tree1->m_right, model0, model1, Rot0, Rot1);
      }
      else if (tree1->is_leaf() && !tree0->is_leaf()) {
          return isIntersection(tree0->m_left, tree1, model0, model1, Rot0, Rot1) || isIntersection(tree0->m_right, tree1, model0, model1, Rot0, Rot1);
      }
      else if (tree0->is_leaf() && tree1->is_leaf()) {
          if (!isIntersectBox(tree0->m_box, tree1->m_box, model0, model1, Rot0, Rot1)) {
              data_list[0].drawBox(tree0->m_box, Eigen::RowVector3d::Random().normalized());
              data_list[1].drawBox(tree1->m_box, Eigen::RowVector3d::Random().normalized());
              return true;
          }
          else return false;
      }
      else {
          if (!isIntersectBox(tree0->m_box, tree1->m_box, model0, model1, Rot0, Rot1)) {
              return (isIntersection(tree0->m_left, tree1->m_left, model0, model1, Rot0, Rot1)
                  || isIntersection(tree0->m_left, tree1->m_right, model0, model1, Rot0, Rot1)
                  || isIntersection(tree0->m_right, tree1->m_left, model0, model1, Rot0, Rot1)
                  || isIntersection(tree0->m_right, tree1->m_right, model0, model1, Rot0, Rot1));
          }
          else return false;
      }
  }
  

} // end namespace
} // end namespace
}
