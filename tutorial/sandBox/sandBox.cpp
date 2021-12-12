#include "tutorial/sandBox/sandBox.h"
#include "igl/edge_flaps.h"
#include "igl/collapse_edge.h"
#include "Eigen/dense"
#include <functional>
//i added
#include <igl/triangle/triangulate.h>
#include <igl/shortest_edge_and_midpoint.h>
#include <igl/parallel_for.h>
#include "igl/opengl/glfw/Viewer.h"
#include "igl/opengl/ViewerData.h"
#include "igl/opengl/glfw/renderer.h"
using namespace std; // i added
using namespace Eigen;  // i added
using namespace igl; // i added



SandBox::SandBox()
{
	

}

void SandBox::Init(const std::string &config)
{
	std::string item_name;
	std::ifstream nameFileout;
	doubleVariable = 0;
	nameFileout.open(config);
	if (!nameFileout.is_open())
	{
		std::cout << "Can't open file "<<config << std::endl;
	}
	else
	{
		int x = 1;
		while (nameFileout >> item_name)
		{
			std::cout << "openning " << item_name << std::endl;
			load_mesh_from_file(item_name);
			data().TranslateInSystem(GetRotation(), Vector3d(x, 0, 0));
			x = - 1.5;
			parents.push_back(-1);
			data().add_points(Eigen::RowVector3d(0, 0, 0), Eigen::RowVector3d(0, 0, 1));
			data().show_overlay_depth = false;
			data().point_size = 10;
			data().line_width = 2;
			data().set_visible(false, 1);
			//data().SetCenterOfRotation(Eigen::Vector3d(1, 0, 0)); added in lecture no need in that
			SetQueue(data().V, data().F);
			data().tree.init(data().V, data().F);
			data().drawBox(data().tree.m_box, Eigen::RowVector3d::Random().normalized());
			//data().setBox();
			
			

		}
		nameFileout.close();
	}
	MyTranslate(Eigen::Vector3d(0, 0, -1), true);
	
	data().set_colors(Eigen::RowVector3d(0.9, 0.1, 0.1));
}

SandBox::~SandBox()
{

}

void SandBox::Animate()
{

	if (isActive)
	{
		//assignment2
		data_list[1].MyTranslate(Eigen::Vector3d(x_direction, y_direction, 0), true);
		Eigen::Matrix4d model0 = data_list[0].MakeTransd().cast<double>();
		Eigen::Matrix4d model1 = data_list[1].MakeTransd().cast<double>();
		Eigen::Matrix3d Rot0 = data_list[0].GetRotation();
		Eigen::Matrix3d Rot1 = data_list[1].GetRotation();


		if (isIntersection(&data_list[0].tree, &data_list[1].tree, model0, model1, Rot0, Rot1)) {
			isActive = !isActive;
			//x_direction = 0;
			//y_direction = 0;
		}
	}
}

/*
		//igl::opengl::glfw::Viewer* scn = renderer->GetScene();
		Eigen::Matrix4d model0;
		Eigen::Matrix4d model1;
		Eigen::Matrix3d Rot0;
		Eigen::Matrix3d Rot1;
		
		igl::AABB<Eigen::MatrixXd, 3>* tree0 = &(data_list[0].tree);
		igl::AABB<Eigen::MatrixXd, 3>* tree1 = &(data_list[1].tree);

		if (scn->move_models) {
			//scn->data_list[1].MyTranslate(Eigen::Vector3d(x_direction, y_direction, 0), false);
			scn->data_list[1].MyTranslate(Eigen::Vector3d(0.01, 0, 0), false);
			//std::cout << "looping-Display";
			model0 = scn->data_list[0].MakeTransd(); //.cast<double>();
			model1 = scn->data_list[1].MakeTransd(); //.cast<double>();
			Rot0 = scn->data_list[0].GetRotation();
			Rot1 = scn->data_list[1].GetRotation();
			scn->isIntersect
			(tree0->m_box, tree1->m_box, model0, model1, Rot0, Rot1);
		}
		*/
		//end assignment2



	




//void SandBox::SetQueue()
void SandBox::SetQueue(Eigen::MatrixXd& V, Eigen::MatrixXi& F)
{
	MatrixXi* Etmp = new MatrixXi(), * EFtmp = new MatrixXi(), * EItmp = new MatrixXi();
	VectorXi* EMAPtmp = new VectorXi();;
	PriorityQueue* Qtmp = new PriorityQueue();
	std::vector<PriorityQueue::iterator> QitTmp;
	edge_flaps(F, *Etmp, *EMAPtmp, *EFtmp, *EItmp);
	//edge_flaps(data().F, *Etmp, *EMAPtmp, *EFtmp, *EItmp);
	QitTmp.resize(Etmp->rows());

	C.push_back(new Eigen::MatrixXd(Etmp->rows(), V.cols())); //the location of the new vertex made from merging
	//C.push_back(new Eigen::MatrixXd(Etmp->rows(), data().V.cols()));
	Eigen::VectorXd costs(Etmp->rows());
	Qtmp->clear();

	//need to implement task 10 
	//const auto& cost_and_placement =
		//[&](const int e,
			//const Eigen::MatrixXd& V,
			//const Eigen::MatrixXi& /*F*/,
			//const Eigen::MatrixXi& E,
			//const Eigen::VectorXi& /*EMAP*/,
			//const Eigen::MatrixXi& /*EF*/,
			//const Eigen::MatrixXi& /*EI*/,
			//double& cost,
			//Eigen::RowVectorXd& p) -> void
 /* {
		for (int v = 0; v < V.size(); v++) {
			Vector3d normal;
			bool found = false;
			Matrix<double, 4, 4> Kp;
			vector<Vector3d>normals = vector<Vector3d>();
			vector<Matrix<double, 4, 4>>ks = vector<Matrix<double, 4, 4>>();
			Matrix<double,4,4> Q = Matrix<double,4,4>();
			for (int f = 0; f < F.size(); f++) {
				for (int z = 0; z < 3; z++) {
					if (F(f,z)= v) {
						normal = data().F_normals.row(f).normalized();
						normals.push_back(normal);
						found = true;
					}	
				}
				if (found) {
					Vector3d vertex = Vector3d(V(v, 0), V(v, 1), V(v, 2));
					double d = normal.transpose() * vertex; //vectors multiplication som how
					Vector4d plane = Vector4d(normal(0), normal(1), normal(2), d);
					Kp = plane * plane.transpose();
					ks.push_back(Kp);
				}
			}
			for (int k = 0; k < ks.size(); k++) {
				Q = Q + ks.at(k);
			}
			Vector4d vExpanded = Vector4d(V(v, 0), V(v, 1), V(v, 2), 1);
			cost = vExpanded.transpose() * Q * vExpanded;
		}
	};
	*/

	

	for (int e = 0; e < Etmp->rows(); e++) {
		double cost = e;
		Eigen::RowVectorXd p(1, 3);
		//cost_and_placement(e, V, F,*Etmp,*EMAPtmp,*EFtmp,*EItmp,cost,p); 
		shortest_edge_and_midpoint(e, V, F, *Etmp, *EMAPtmp, *EFtmp, *EItmp, cost, p);
		C[selected_data_index]->row(e) = p;
		QitTmp[e] = Qtmp->insert(std::pair<double, int>(cost, e)).first;
	}

	EMAP.push_back(EMAPtmp);
	E.push_back(Etmp);
	EF.push_back(EFtmp);
	EI.push_back(EItmp);
	Q.push_back(Qtmp);
	Qit.push_back(QitTmp);
	num_collapsed.push_back(0);

}


void SandBox::pre_draw() {
	// If animating then collapse 5% of edges
	if (!Q[selected_data_index]->empty())
	{
		bool something_collapsed = false;
		// collapse edge
		const int max_iter = std::ceil(0.05 * (Q[selected_data_index]->size())); //maybe 0.005
		for (int j = 0; j < max_iter; j++)
		{
			if (!collapse_edge
			(shortest_edge_and_midpoint, data().V, data().F, *E[selected_data_index], *EMAP[selected_data_index], *EF[selected_data_index], *EI[selected_data_index], *Q[selected_data_index], Qit[selected_data_index], *C[selected_data_index]))
			{
				break;
			}
			something_collapsed = true;
			num_collapsed.at(selected_data_index)++;
		}

		if (something_collapsed)
		{
			//data().clear();
			data().set_mesh(data().V, data().F);
			data().set_face_based(true);
			data().dirty = 157;
		}
	}
};









