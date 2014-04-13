#include <Eigen/StdVector>
#include <tr1/random>
#include <iostream>
#include <stdint.h>
#include <tr1/unordered_set>

#include "g2o/core/graph_optimizer_sparse.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/solver.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/core/structure_only_solver.h"

#include "NonLinearLS.h"

#include "tokenizer.h"

#include <iostream>
#include <fstream>


const int ITERACIONES = 200;


using namespace Eigen;
using namespace std;


// CARGAR EL GROUND TRUTH
/*
typedef struct
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Vector6 cal;
  Matrix4 cam;
  vector < Vector2, aligned_allocator<Vector2> > pto2D;
  vector < Vector3, aligned_allocator<Vector3> > pto3D;
  Matrix4 solMatlab;
} Datos;
*/

void loadData(Data &dat)
{
   Token t;
   if(t.abrirFichero("deDorian.mat") != Token::ok)
   {
      cerr << "INVALID FILE\n";
      exit (-1);
   }

   int caso = 0;
   while(t.leeLinea() == Token::ok)
   {
      if(t.tokenizar(" \t") == 0)
      {
        caso++;
        continue;
      }

      Vector2 pto2;
      Vector3 pto3;

      cerr << setprecision(10) << fixed;
      switch(caso)
      {
         case 0: // Calibracion
            for(int i = 0; i < t.getnPalabras(); i++)
            {
               dat.cal(i) = atof(t.token(i));
            }
            //cerr << "Calibracion: " << dat.cal.transpose() << endl;
            break;
         case 1: // Camara
            dat.cam = Matrix4d::Identity();
            for(int i = 0; i < t.getnPalabras(); i++)
            {
               int f = i%3, c = i/3;
               dat.cam(f,c) = atof(t.token(i));
            }
            cerr << "cam: " << dat.cam << endl;
            break;
         case 2: // Puntos 2D
            //cerr << "ptos2D:\n";
            for(int i = 0; i < t.getnPalabras(); i++)
            {
               pto2(i) = atof(t.token(i));
            }
            //cerr << pto2 << endl;
            dat.pto2D.push_back(pto2);
            break;
         case 3: // Puntos 3D
            //cerr << "ptos3D:\n";
            for(int i = 0; i < t.getnPalabras(); i++)
            {
               pto3(i) = atof(t.token(i));
            }
            //cerr << pto3 << endl;
            dat.pto3D.push_back(pto3);
            break;
         case 4: // sol Matlab
            dat.solMatlab = Matrix4d::Identity();
            for(int i = 0; i < t.getnPalabras(); i++)
            {
               int f = i%3, c = i/3;
               dat.solMatlab(f,c) = atof(t.token(i));
            }
            //cerr << "mat: " << dat.solMatlab << endl;
         default:
            ;
      }
   }
}


/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/


int main(int argc, const char* argv[])
{

  Data dat;
  loadData(dat);

  poseOptimization(dat);
  

#if 0
/******************************************************************************/
// Configuracion del resolvedor

  g2o::SparseOptimizer optimizer;
  optimizer.setMethod(g2o::SparseOptimizer::LevenbergMarquardt);
  optimizer.setVerbose(true);

  g2o::BlockSolverX::LinearSolverType * linearSolver;
//  if (DENSE)
//  {
//     cerr << "MUY DENSO\n";
     linearSolver =
       new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();
//  }else
  /*{
    linearSolver =
       new g2o::LinearSolverCholmod<g2o::BlockSolverX::PoseMatrixType>();
       //new g2o::LinearSolverCSparse<g2o::BlockSolverX::PoseMatrixType>();
       //new g2o::LinearSolverPCG<g2o::BlockSolverX::PoseMatrixType>();
//  }*/


  g2o::BlockSolverX * solver_ptr =
     new g2o::BlockSolverX(&optimizer,linearSolver);


  optimizer.setSolver(solver_ptr);

/******************************************************************************/

  int vertex_id = 0;


//////////////
/// CAMARA ///
//////////////
  g2o::VertexSE3AxisAngle * v_se3 = new g2o::VertexSE3AxisAngle(dat.cal, dat.cam);

  v_se3->setId(vertex_id);

  optimizer.addVertex(v_se3);
  vertex_id++;


//////////////
/// PUNTOS ///
//////////////
  for (size_t i=0; i < dat.pto3D.size(); ++i)
  {
    g2o::VertexPointXYZ * v_p = new g2o::VertexPointXYZ();

    v_p->setId(vertex_id);
    v_p->setMarginalized(true);
    v_p->estimate() = dat.pto3D[i];

    v_p->setFixed(true);

    g2o::EdgeProjectXYZ2UV * e = new g2o::EdgeProjectXYZ2UV();
    e->vertices()[0] = dynamic_cast<g2o::OptimizableGraph::Vertex*>(v_se3);
    e->vertices()[1] = dynamic_cast<g2o::OptimizableGraph::Vertex*>(v_p);
    e->measurement() = dat.pto2D[i];
    e->inverseMeasurement() = -dat.pto2D[i];

    e->information() = Matrix2d::Identity();
    e->setRobustKernel(false);
    e->setHuberWidth(1);

    optimizer.addEdge(e);
    optimizer.addVertex(v_p);
    vertex_id ++;
  }


////////////////////
/// OPTIMIZACION ///
////////////////////

  cout << "\n ----------------------------------- \n\n";

  optimizer.initializeOptimization();
  optimizer.setVerbose(true);

  optimizer.optimize(ITERACIONES);

  cerr << "Camara final: " << v_se3->getCamera() << endl;
  cerr << "Camara Matlab: " << dat.solMatlab << endl;
  cerr << "AAAA " << dat.cam << endl;
#endif
}



