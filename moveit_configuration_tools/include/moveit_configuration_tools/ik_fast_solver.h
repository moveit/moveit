#include <Eigen/Geometry>
#include <float.h>

typedef double IKReal;

namespace kinematics_constraint_aware {
class ik_solver_base {
public:
  virtual int solve(Eigen::Affine3d &pose, const std::vector<double> &ik_seed_state) = 0;
  virtual void getSolution(int i, std::vector<double> &solution) = 0;
  virtual void getClosestSolution(const std::vector<double> &ik_seed_state, std::vector<double> &solution) = 0;
};
template <class  T> class ikfast_solver: public ik_solver_base{
public:
  typedef bool (*ik_type)(const IKReal* eetrans, const IKReal* eerot, const IKReal* pfree, std::vector<T>& vsolutions);
  ikfast_solver(ik_type ik,int numJoints):ik(ik),numJoints(numJoints) {}
  virtual int solve(Eigen::Affine3d &pose, const std::vector<double> &vfree){
      
    solutions.clear();

    double vals[9];
    vals[0] = pose.rotation()(0,0);
    vals[1] = pose.rotation()(0,1);
    vals[2] = pose.rotation()(0,2);
    vals[3] = pose.rotation()(1,0);
    vals[4] = pose.rotation()(1,1);
    vals[5] = pose.rotation()(1,2);
    vals[6] = pose.rotation()(2,0);
    vals[7] = pose.rotation()(2,1);
    vals[8] = pose.rotation()(2,2);

    double trans[3];
    trans[0] = pose.translation().x();
    trans[1] = pose.translation().y();
    trans[2] = pose.translation().z();

    ik(trans, vals, vfree.size() > 0 ? &vfree[0] : NULL, solutions);
      
    return solutions.size();
  }
  virtual void getSolution(int i, std::vector<double> &solution){
    solution.clear();
    std::vector<IKReal> vsolfree(solutions[i].GetFree().size());
    solution.clear();
    solution.resize(numJoints);
    solutions[i].GetSolution(&solution[0],vsolfree.size()>0?&vsolfree[0]:NULL);
    // std::cout << "solution " << i << ":" ;
    // for(int j=0;j<numJoints; ++j)
    //   std::cout << " " << solution[j];
    // std::cout << std::endl;
	  
    //ROS_ERROR("%f %d",solution[2],vsolfree.size());
  }
  double harmonize(const std::vector<double> &ik_seed_state, std::vector<double> &solution){
    double dist_sqr = 0;
    std::vector<double> ss = ik_seed_state;
    for(size_t i=0; i< ik_seed_state.size(); ++i){
      while(ss[i] > 2*M_PI) {
        ss[i] -= 2*M_PI;
      }
      while(ss[i] < 2*M_PI) {
        ss[i] += 2*M_PI;
      }
      while(solution[i] > 2*M_PI) {
        solution[i] -= 2*M_PI;
      }
      while(solution[i] < 2*M_PI) {
        solution[i] += 2*M_PI;
      }
      dist_sqr += fabs(ik_seed_state[i] - solution[i]);
    }
    return dist_sqr;
  }

  double harmonize_old(const std::vector<double> &ik_seed_state, std::vector<double> &solution){
    double dist_sqr = 0;
    for(size_t i=0; i< ik_seed_state.size(); ++i){
      double diff = ik_seed_state[i] - solution[i];
      if( diff > M_PI ) solution[i]+=2*M_PI; 
      else if (diff < -M_PI) solution[i]-=2*M_PI;
      diff = ik_seed_state[i] - solution[i];
      dist_sqr += fabs(diff);
    }
    return dist_sqr;
  }
  
  // virtual void getOrderedSolutions(const std::vector<double> &ik_seed_state, 
  //                                  std::vector<std::vector<double> >& solutions){
  //   std::vector<double> 
  //   double mindist = 0;
  //   int minindex = -1;
  //   std::vector<double> sol;
  //   for(size_t i=0;i<solutions.size();++i){
  //     getSolution(i,sol);
  //     double dist = harmonize(ik_seed_state, sol);
  //     //std::cout << "dist[" << i << "]= " << dist << std::endl;
  //     if(minindex == -1 || dist<mindist){
  //       minindex = i;
  //       mindist = dist;
  //     }
  //   }
  //   if(minindex >= 0){
  //     getSolution(minindex,solution);
  //     harmonize(ik_seed_state, solution);
  //     index = minindex;
  //   }
    
  // }

  virtual void getClosestSolution(const std::vector<double> &ik_seed_state, std::vector<double> &solution){
    double mindist = DBL_MAX;
    int minindex = -1;
    std::vector<double> sol;
    for(size_t i=0;i<solutions.size();++i){
      getSolution(i,sol);
      double dist = harmonize(ik_seed_state, sol);
      ROS_INFO_STREAM("Dist " << i << " dist " << dist);
      //std::cout << "dist[" << i << "]= " << dist << std::endl;
      if(minindex == -1 || dist<mindist){
        minindex = i;
        mindist = dist;
      }
    }
    if(minindex >= 0){
      getSolution(minindex,solution);
      harmonize(ik_seed_state, solution);
    }
  }
    
    
    
private:
  ik_type ik;
  std::vector<T> solutions;
  int numJoints;
};
}
