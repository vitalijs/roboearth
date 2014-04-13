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

#include <types_g2o_ptam.h>

#include "tokenizer.h"

#include <iostream>
#include <fstream>

//const int W = 640;
//const int H = 480;

unsigned int CAMARAS = 700;
unsigned int PUNTOS;

const int ITERACIONES = 20;

const bool CAL_CORRECTA = false;

const bool FIJAS = true;
const unsigned int NUM_FIJAS = 1;//1;
const bool RUIDO_CAM = false;

const bool PTOS_FIJOS = false;
const bool RUIDO_PTOS = false;


using namespace Eigen;
using namespace std;


class Sample
{

  static tr1::ranlux_base_01 gen_real;
  static tr1::mt19937 gen_int;
public:
  static int uniform(int from, int to);

  static double uniform();

  static double gaussian(double sigma);
};


tr1::ranlux_base_01 Sample::gen_real;
tr1::mt19937 Sample::gen_int;

int Sample::uniform(int from, int to)
{
  tr1::uniform_int<int> unif(from, to);
  int sam = unif(gen_int);
  return  sam;
}

double Sample::uniform()
{
  std::tr1::uniform_real<double> unif(0.0, 1.0);
  double sam = unif(gen_real);
  return  sam;
}

double Sample::gaussian(double sigma)
{
  if(sigma < 1e-10) return 0;
  std::tr1::normal_distribution<double> gauss(0.0, sigma);
  double sam = gauss(gen_real);
  return  sam;
}


// CARGAR EL GROUND TRUTH
typedef struct
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  int h, w;
  Vector4 cal;
  vector < Matrix4, aligned_allocator<Matrix4> > cam;
  vector < Vector3, aligned_allocator<Vector3> > pto;
} GroundTruth;

void GT(GroundTruth &gt)
{
   Token t;
   if(t.abrirFichero("cavidad.mat") != Token::ok)
   {
      cerr << "FICHERO INVALIDO\n";
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

      Matrix4 cam;
      cam = Matrix4d::Identity();
      Vector3 pto;
      cerr << setprecision(10) << fixed;
      switch(caso)
      {
         case 0: // Calibracion
            gt.w = atoi(t.token(0));
            gt.h = atoi(t.token(1));
            for(int i = 0; i < t.getnPalabras()-2; i++)
            {
               gt.cal(i) = atof(t.token(i+2));
            }
            break;
         case 1: // Camaras
            for(int i = 0; i < t.getnPalabras(); i++)
            {
               int f = i%3, c = i/3;
               //cerr << "i: " << i << "  (f,c): (" << f << "," << c << ")\n";
               //cam(i) = atof(t.token(i));
               cam(f,c) = atof(t.token(i));
            }
            //cerr << fixed << setprecision(5) << cam << endl;
            gt.cam.push_back(cam);
            break;
         case 2: // Puntos
            for(int i = 0; i < t.getnPalabras(); i++)
            {
               pto(i) = atof(t.token(i));
            }
            gt.pto.push_back(pto);
         default:
            ;
      }
   }
   if (CAMARAS > gt.cam.size()) CAMARAS = gt.cam.size();
   PUNTOS = gt.pto.size();
}
///////////////////


// MATLAB
typedef Matrix<double,2,1> Vector2;

typedef struct
{
   EIGEN_MAKE_ALIGNED_OPERATOR_NEW
   int n, fija;
   //SE3AxisAngle c;
   //Vector6 c;
   Matrix4 Ecw;
} CamaraMatlab;

typedef struct
{
   EIGEN_MAKE_ALIGNED_OPERATOR_NEW
   int n;
   //VertexPointXYZ p;
   Vector3 p;
} PuntoMatlab;

typedef struct
{
   EIGEN_MAKE_ALIGNED_OPERATOR_NEW
   int cam, pto, espurio;
   Vector2d m;
} MedicionMatlab;
/////////////////


/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/


int main(int argc, const char* argv[])
{
  if (argc<2)
  {
    cout << "\nPlease type:\n"
         << "ba_demo [PIXEL_NOISE] [OUTLIER RATIO] [ROBUST_KERNEL] "
         << "[STRUCTURE_ONLY] [DENSE]\n\n"
         << "PIXEL_NOISE: noise in image space (E.g.: 1)\n"
         << "OUTLIER_RATIO: probability of spuroius observation  "
         << "(default: 0.0)\n"
         << "ROBUST_KERNEL: use robust kernel (0 or 1; default: 0==false)\n"
         << "STRUCTURE_ONLY: performe structure-only BA to get better point "
         << "initializations (0 or 1; default: 0==false)\n"
         << "DENSE: Use dense solver (0 or 1; default: 0==false)\n\n"
         << "Note, if OUTLIER_RATIO is above 0, ROBUST_KERNEL should be set to "
         << "1==true.\n\n";
    exit(0);
  }


  GroundTruth gt;
  GT(gt);

  double PIXEL_NOISE = atof(argv[1]);

  double OUTLIER_RATIO = 0.0;

  if (argc>2)
  {
    OUTLIER_RATIO = atof(argv[2]);
  }

  bool ROBUST_KERNEL = false;
  if (argc>3)
  {
    ROBUST_KERNEL = atof(argv[3]);
  }
  bool STRUCTURE_ONLY = false;
  if (argc>4)
  {
    STRUCTURE_ONLY = atof(argv[4]);
  }

  bool DENSE = false;
  if (argc>5)
  {
    DENSE = atof(argv[5]);
  }

  cout << "PIXEL_NOISE: " <<  PIXEL_NOISE << endl
       << "OUTLIER_RATIO: " << OUTLIER_RATIO<<  endl
       << "ROBUST_KERNEL: " << ROBUST_KERNEL << endl
       << "STRUCTURE_ONLY: " << STRUCTURE_ONLY<< endl
       << "DENSE: "<<  DENSE << endl;



/******************************************************************************/
// Configuracion del resolvedor

  g2o::SparseOptimizer optimizer;
  optimizer.setMethod(g2o::SparseOptimizer::LevenbergMarquardt);
  optimizer.setVerbose(true);

  g2o::BlockSolverX::LinearSolverType * linearSolver;
  if (DENSE)
  {
     cerr << "MUY DENSO\n";
     linearSolver =
       new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();
  }else
  {
    linearSolver =
       new g2o::LinearSolverCholmod<g2o::BlockSolverX::PoseMatrixType>();
       //new g2o::LinearSolverCSparse<g2o::BlockSolverX::PoseMatrixType>();
       //new g2o::LinearSolverPCG<g2o::BlockSolverX::PoseMatrixType>();
  }


  g2o::BlockSolverX * solver_ptr =
     new g2o::BlockSolverX(&optimizer,linearSolver);


  optimizer.setSolver(solver_ptr);

/******************************************************************************/

  int vertex_id = 0;


  // MATLAB
  CamaraMatlab*   cM;
  PuntoMatlab*    pM;
  MedicionMatlab* mM;
  vector<CamaraMatlab*>   camGT, cam0, camF;
  vector<PuntoMatlab*>    ptoGT, pto0, ptoF;
  vector<MedicionMatlab*> med,  medGT;
  ///////////////


///////////////////
/// CALIBRACION ///
///////////////////

  g2o::VertexKFOV4 *vK0, *vKF, *vKGT;

  vKGT = new g2o::VertexKFOV4(gt.cal[0], gt.cal[1], gt.cal[2], gt.cal[3]);
  cerr << "Cal: " << vKGT->estimate().transpose() << endl;
  if(CAL_CORRECTA)
  {
     vK0 = new g2o::VertexKFOV4(gt.cal[0], gt.cal[1], gt.cal[2], gt.cal[3]);
     vK0->setFixed(true);
     vKF = new g2o::VertexKFOV4(gt.cal[0], gt.cal[1], gt.cal[2], gt.cal[3]);
     vKF->setFixed(true);
  }else
  {
//     double f = gt.w/(2*tan(60*M_PI/180));
//     vK0 = new g2o::VertexKFOV5(f, f, gt.w>>1, gt.h>>1, 60*M_PI/180);
//     vKF = new g2o::VertexKFOV5(f, f, gt.w>>1, gt.h>>1, 60*M_PI/180);
     Vector4 cal;
     cal(0) = /*Sample::gaussian(0.05*gt.cal[0]) +*/ 1.05 * gt.cal[0];
     cal(1) = /*Sample::gaussian(0.05*gt.cal[1]) +*/ 1.05 * gt.cal[1];
     cal(2) = /*Sample::gaussian(0.05*gt.cal[2]) +*/ 1.05 * gt.cal[2];
     cal(3) = /*Sample::gaussian(0.05*gt.cal[3]) +*/ 1.05 * gt.cal[3];
     vK0 = new g2o::VertexKFOV4(cal[0], cal[1], cal[2], cal[3]);
     vKF = new g2o::VertexKFOV4(cal[0], cal[1], cal[2], cal[3]);

     vK0->setFixed(false);
     vKF->setFixed(false);
  }
  vKF->setId(vertex_id);
  optimizer.addVertex(vKF);
  vertex_id++;


////////////////////////////////////////////////////////////////////////////////

///////////////
/// CAMARAS ///
///////////////
  vector<Matrix4, aligned_allocator<Matrix4> > true_cameras;
  for (size_t i=0; i<CAMARAS; ++i)
  {
    Matrix4 Ecw;
    Ecw = gt.cam[i];
    Vector6 mu;
    true_cameras.push_back(Ecw);

    // MATLAB
    cM = new CamaraMatlab;
    cM->Ecw  = Ecw;
    ///////////////

    if(RUIDO_CAM && i > 1)     // ANIADIR RUIDO A mu
    {
       mu(0) = Sample::gaussian(0.2);
       mu(1) = Sample::gaussian(0.2);
       mu(2) = Sample::gaussian(0.2);
       mu(3) = Sample::gaussian(0.09);
       mu(4) = Sample::gaussian(0.09);
       mu(5) = Sample::gaussian(0.09);
       Ecw = SE3AxisAngle::exp(mu) * Ecw;
       //mu.Zero();
    }

    //mu.Zero();
    //mu(0) = i*0.04-1.; mu(1) = mu(2) = mu(3) = mu(4) = mu(5) = 0;
    //mu(0) = (int(i)-1)*2.5; mu(1) = 0; mu(2) = -5;
    //mu(3) = mu(4) = mu(5) = 0;

    g2o::VertexSE3AxisAngle * v_se3 = new g2o::VertexSE3AxisAngle(Ecw);

    v_se3->setId(vertex_id);
    //v_se3->setToOrigin();

    if (FIJAS && i<NUM_FIJAS)   v_se3->setFixed(true);

    optimizer.addVertex(v_se3);
    vertex_id++;

    // MATLAB
    cM->n    = v_se3->id();
    cM->fija = v_se3->fixed();
    camGT.push_back(cM);

    cM = new CamaraMatlab;
    cM->n    = v_se3->id();
    cM->fija = v_se3->fixed();
    cM->Ecw  = Ecw;
    cam0.push_back(cM);
    /////////////

  }

////////////////////////////////////////////////////////////////////////////////

/////////////////////
/// PUNTOS REALES ///
/////////////////////
  vector<Vector3> true_points;
  for (size_t i=0; i < PUNTOS; ++i)   true_points.push_back(gt.pto[i]);


////////////////////////////////////////////////////////////////////////////////

///////////////////////
/// PUNTOS RUIDOSOS ///
///////////////////////

  int point_id = vertex_id;

  int point_num = 0;
  double sum_diff2 = 0;

  cout << endl;
  tr1::unordered_map<int,int> pointid_2_trueid;
  tr1::unordered_set<int> inliers;

  for (size_t i=0; i<true_points.size(); ++i)
  {
    g2o::VertexPointXYZ * v_p = new g2o::VertexPointXYZ();

    // Estimacion inicial del punto 3 (con error)
    // Es lo que hay que optimizar
    v_p->setId(point_id);
    v_p->setMarginalized(true);
    v_p->estimate() = true_points[i];
    if(RUIDO_PTOS)
      v_p->estimate() += Vector3
          (Sample::gaussian(0.5), Sample::gaussian(0.5), Sample::gaussian(0.5));

    v_p->setFixed(PTOS_FIJOS);

    ++point_id;


    // Comprobacion de que el pto es visto por mas de 2 camaras
    int num_obs = 0;
    for (size_t j = 0; j < true_cameras.size(); ++j)
    {
      g2o::VertexSE3AxisAngle *vc = new g2o::VertexSE3AxisAngle(gt.cam[j]);
      //vc->setToOrigin();
      Vector2d z =
         dynamic_cast<g2o::VertexSE3AxisAngle*>
            (vc)->map(true_points.at(i), vKGT->estimate());
      delete vc;
      if (z[0]>=0 && z[1]>=0 && z[0]<gt.w && z[1]<gt.h)   if(++num_obs > 1) break;
    }
    if (num_obs<2)   continue;


    // MATLAB
    pM    = new PuntoMatlab;
    pM->n = v_p->id();
    pM->p = true_points[i];
    ptoGT.push_back(pM);

    pM    = new PuntoMatlab;
    pM->n = v_p->id();
    pM->p = v_p->estimate(); //true_points[i];
    pto0.push_back(pM);
    ////////////

    // Buscar las camaras en las que se ve y calcular su respectiva medicion.
    // Si el pto es considerado en una camara como espurio => ya se queda como
    // espurio y en el tratamiento de los errores no se tiene en cuenta pero si
    // que cuenta a la hora de repartir el error de los ptos no espurios.
    // Creo que esta politica esta MAL. La he modificado para que no se tengan
    // en cuenta en el calculo del error medio :) y aun asi sigue estando mal
    // que si un pto en una medicion es espurio lo sea en las demas
    bool inlier = true;
    for (size_t j = 0; j < true_cameras.size(); ++j)
    {
      g2o::VertexSE3AxisAngle *vc = new g2o::VertexSE3AxisAngle(gt.cam[j]);
      //vc->setToOrigin();
      Vector2d z =
         dynamic_cast<g2o::VertexSE3AxisAngle*>
            (vc)->map(true_points.at(i), vKGT->estimate());
      delete vc;

      if (z[0] >= 0 && z[1] >= 0 && z[0] < gt.w && z[1] < gt.h)
      {

        // MATLAB
        mM = new MedicionMatlab;
        mM->cam     = optimizer.vertices().find(j+1)->second->id();
        mM->pto     = v_p->id();
        mM->espurio = 0;
        mM->m       = z;
        medGT.push_back(mM);
        ////////////////

        double sam = Sample::uniform();
        if (sam < OUTLIER_RATIO)
        {
          z = Vector2d(Sample::uniform(0, gt.w), Sample::uniform(0, gt.h));
          inlier= false;
        }

        // Medida + Error de medida
        z += Vector2d(Sample::gaussian(PIXEL_NOISE),
                      Sample::gaussian(PIXEL_NOISE));


        // MATLAB
        mM = new MedicionMatlab;
        mM->cam     = optimizer.vertices().find(j+1)->second->id();
        mM->pto     = v_p->id();
        mM->espurio = inlier?0:1;
        mM->m       = z;
        med.push_back(mM);
        ////////////////


        g2o::EdgeKProjectXYZ2UV * e = new g2o::EdgeKProjectXYZ2UV();
        e->vertices()[0] = dynamic_cast<g2o::OptimizableGraph::Vertex*>(vKF);
        e->vertices()[1] =
           dynamic_cast<g2o::OptimizableGraph::Vertex*>
              (optimizer.vertices().find(j+1)->second);  //j+1
        e->vertices()[2] = dynamic_cast<g2o::OptimizableGraph::Vertex*>(v_p);

        e->measurement() = z;
        e->inverseMeasurement() = -z;
        e->information() = Matrix2d::Identity()/(PIXEL_NOISE*PIXEL_NOISE);
        e->setRobustKernel(ROBUST_KERNEL);
        e->setHuberWidth(1);
        optimizer.addEdge(e);
      }
    }

    if (inlier)
    {
      inliers.insert(v_p->id());
      Vector3 diff = v_p->estimate() - true_points[i];

      sum_diff2 += diff.dot(diff);
      ++point_num;
    }

    optimizer.addVertex(v_p);

    pointid_2_trueid.insert(make_pair(v_p->id(),i));
  }








////////////////////////////////////////////////////////////////////////////////

////////////////////
/// OPTIMIZACION ///
////////////////////

  cout << "\n ----------------------------------- \n\n";

  optimizer.initializeOptimization();
  optimizer.setVerbose(true);

  if (STRUCTURE_ONLY)
  {
    // Deja fijas las camaras y solo optimiza los ptos
    cout << "Performing structure-only BA:"   << endl;
    g2o::StructureOnlySolver<3> structure_only_ba;
    structure_only_ba.setVerbose(true);
    structure_only_ba.calc(optimizer.vertices(), 10);
  }

  cout << "\nPerforming full BA:" << endl;
  optimizer.optimize(ITERACIONES);


////////////////////////////////////////////////////////////////////////////////

///////////////
/// ERRORES ///
///////////////

  cout << "\nPoint error before optimisation (inliers only): "
       << sqrt(sum_diff2/point_num) << endl;


  // Calculo del error tras las optimizacion
  point_num = 0;
  sum_diff2 = 0;
  for (tr1::unordered_map<int,int>::iterator
       it  = pointid_2_trueid.begin();
       it != pointid_2_trueid.end();
       ++it)
  {
    g2o::HyperGraph::VertexIDMap::iterator v_it =
       optimizer.vertices().find(it->first);

    if (v_it == optimizer.vertices().end())
    {
      cerr << "Vertex " << it->first << " not in graph!" << endl;
      exit(-1);
    }

    g2o::VertexPointXYZ * v_p =
       dynamic_cast< g2o::VertexPointXYZ * > (v_it->second);

    if (v_p == 0)
    {
      cerr << "Vertex " << it->first << "is not a PointXYZ!" << endl;
      exit(-1);
    }

    // MATLAB
    pM    = new PuntoMatlab;
    pM->n = v_p->id();
    pM->p = v_p->estimate(); //true_points[i];
    ptoF.push_back(pM);
    ////////////


    Vector3 diff = v_p->estimate() - true_points[it->second];

    if (inliers.find(it->first)==inliers.end())   continue;

    sum_diff2 += diff.dot(diff);
    ++point_num;
  }

  cout << "Point error after optimisation (inliers only): "
       << sqrt(sum_diff2/point_num) << endl << endl;



  // Ploteo de las camaras optimizadas
  for (size_t j=0; j<true_cameras.size(); ++j)
  {
    g2o::HyperGraph::VertexIDMap::iterator v_it = optimizer.vertices().find(j+1);
    g2o::VertexSE3AxisAngle * v_c =
       dynamic_cast< g2o::VertexSE3AxisAngle * > (v_it->second);

    // MATLAB
    cM = new CamaraMatlab;
    cM->n      = v_c->id();
    cM->fija   = v_c->fixed();
//    cM->c    = v_c->estimate().getAxisAngle().transpose();
    cM->Ecw    = v_c->getCamera();
    camF.push_back(cM);
    /////////////
  }


  // Ploteo de la calibracion optimizadas
//  g2o::HyperGraph::VertexIDMap::iterator v_it = optimizer.vertices().find(0);
//  vKF = dynamic_cast< g2o::VertexKFOV5 * > (v_it->second);

////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////
/// GENERACION DEL FICHERO  MATLAB ///
//////////////////////////////////////

  ofstream file ("g2o.m", ofstream::out | ofstream::trunc);

  if (!file) return -1;

  file << setprecision(15) << fixed;


  file << "kGT = [" << vKGT->estimate().transpose() << "];\n\n\n";


  file << "camsGT = [\n";
  for(size_t c = 0; c < camGT.size(); c++)
  {
     file << camGT[c]->n << " " << camGT[c]->fija << " "
          << camGT[c]->Ecw.col(0).transpose() << " "
          << camGT[c]->Ecw.col(1).transpose() << " "
          << camGT[c]->Ecw.col(2).transpose() << " "
          << camGT[c]->Ecw.col(3).transpose() << endl;
  }
  file << "];\n\n\n";


  file << "ptosGT = [\n";
  for(size_t p=0; p < ptoGT.size(); p++)
  {
     file << ptoGT[p]->n << " " << ptoGT[p]->p.transpose() << endl;
  }
  file << "];\n\n\n";



  file << "medsGT = [\n";
  for(size_t m=0; m < medGT.size(); m++)
  {
     file << medGT[m]->cam << " " << medGT[m]->pto << " "
          << medGT[m]->espurio << " " << medGT[m]->m.transpose() << endl;
  }
  file << "];\n\n\n";



  file << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n"
       << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n"
       << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n";



  file << "k0 = [" << vK0->estimate().transpose() << "];\n\n\n";


  file << "cams0 = [\n";
  for(size_t c = 0; c < cam0.size(); c++)
  {
     file << cam0[c]->n << " " << cam0[c]->fija << " "
          << cam0[c]->Ecw.col(0).transpose() << " "
          << cam0[c]->Ecw.col(1).transpose() << " "
          << cam0[c]->Ecw.col(2).transpose() << " "
          << cam0[c]->Ecw.col(3).transpose() << endl;
  }
  file << "];\n\n\n";


  file << "ptos0 = [\n";
  for(size_t p=0; p < pto0.size(); p++)
  {
     file << pto0[p]->n << " " << pto0[p]->p.transpose() << endl;
  }
  file << "];\n\n\n";


  file << "meds0 = [\n";
  for(size_t m=0; m < med.size(); m++)
  {
     file << med[m]->cam << " " << med[m]->pto << " "
          << med[m]->espurio << " " << med[m]->m.transpose() << endl;
  }
  file << "];\n\n\n";



  file << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n"
       << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n"
       << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%\n";



  file << "kF = [" << vKF->estimate().transpose() << "];\n\n\n";


  file << "camsF = [\n";
  for(size_t c = 0; c < camF.size(); c++)
  {
     file << camF[c]->n << " " << camF[c]->fija << " "
          << camF[c]->Ecw.col(0).transpose() << " "
          << camF[c]->Ecw.col(1).transpose() << " "
          << camF[c]->Ecw.col(2).transpose() << " "
          << camF[c]->Ecw.col(3).transpose() << endl;
  }
  file << "];\n\n\n";


  file << "ptosF = [\n";
  for(size_t p=0; p < ptoF.size(); p++)
  {
     file << ptoF[p]->n << " " << ptoF[p]->p.transpose() << endl;
  }
  file << "];\n\n\n";

  file.close();

  cerr << "kGT: " << vKGT->estimate().transpose() << endl
       << "k0 : " << vK0->estimate().transpose() << endl
       << "kF : " << vKF->estimate().transpose() << endl
       << "-------------------------------------------\n"
       << "kF - kGT : "
       << vKGT->estimate().transpose() -vKF->estimate().transpose() << endl;
}



