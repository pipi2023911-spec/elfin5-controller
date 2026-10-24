// 真实Eigen/求解器回归；读取失败、空/截断文件、NaN及未收敛均不能冒充PASS。
#include "elfin5_qp_controller/box_qp_solver.h"
#include <Eigen/Core>
#include <chrono>
#include <fstream>
#include <sstream>
#include <iostream>
#include <stdexcept>
#include <limits>
#include <cmath>
#include <algorithm>
#include <vector>
using elfin5_qp_controller::BoxQPSolver;
using elfin5_qp_controller::QPSolution;
namespace {
int checks=0;
void check(bool ok,const std::string& message) {++checks;if(!ok)throw std::runtime_error(message);}
void near(const Eigen::VectorXd& x,const Eigen::VectorXd& expected,double tol=1e-8) {
  check(x.size()==expected.size() && (x-expected).norm()<=tol,"解析最优解不符");
}
void selfTests() {
  Eigen::MatrixXd H=Eigen::MatrixXd::Identity(2,2);
  Eigen::VectorXd g=Eigen::VectorXd::Zero(2),l=Eigen::VectorXd::Constant(2,-1),u=Eigen::VectorXd::Ones(2),a(2);
  BoxQPSolver solver(100,1e-7);a<<1,.01;
  auto s=solver.solve(H,g,l,u,g,&a,1.0099);
  check(s.success&&s.converged,"窄角可行问题不得误拒绝");Eigen::Vector2d expected(1,.99);near(s.x,expected);
  a<<-1,1;g<<-2,0;l.setZero();
  s=solver.solve(H,g,l,u,l,&a,0);check(s.converged,"非最优固定点反例");near(s.x,Eigen::Vector2d(1,1));
  a<<1,1;s=solver.solve(H,g,l,u,l,&a,1.5);check(s.converged,"固定八次投影反例");near(s.x,Eigen::Vector2d(1,.5));
  s=solver.solve(H,g,l,u,l,&a,2.5);check(!s.success,"真不可行必须拒绝");
  g.setZero();s=solver.solve(H,g,l,u,l,&a,2);check(s.converged,"盒支撑面边界必须可解");near(s.x,u);
  a.setZero();s=solver.solve(H,g,l,u,l,&a,1);check(!s.success,"零法向正偏置不可行");
  for(double b:{0.,-1.}) {s=solver.solve(H,g,l,u,l,&a,b);check(s.converged,"零法向恒真约束");near(s.x,l);}
  Eigen::MatrixXd one=Eigen::MatrixXd::Identity(1,1);
  Eigen::VectorXd z=Eigen::VectorXd::Zero(1),ones=Eigen::VectorXd::Ones(1),n(1),want(1);want<<.5;
  for(double scale:{1e-200,5e-8,1.,1e100,1e200}) {
    n<<scale;s=solver.solve(one,z,z,ones,ones,&n,.5*scale);
    check(s.converged,"等价缩放半空间应收敛");near(s.x,want);
  }
  n<<1;s=solver.solve(one,z,z,ones,ones,&n,1.+1e-8);check(!s.success,"略超支撑面也不可行");
  Eigen::VectorXd negative=Eigen::VectorXd::Constant(1,-1);
  s=solver.solve(one,negative,z,z,z);check(s.converged,"固定变量l=u");near(s.x,z);
  Eigen::MatrixXd ill=H;ill(1,1)=1e-10;g<<0,-.4e-10;
  s=solver.solve(ill,g,l,u,l);check(s.converged,"弱曲率/零空间解");near(s.x,Eigen::Vector2d(0,.4));
  g<<-4,2;s=BoxQPSolver(1,1e-7).solve(2*H,g,-u,u,l);
  check(s.iterations<=1&&!s.converged&&s.success,"总预算与可行未收敛须分开");
  g.setZero();a.setOnes();s=BoxQPSolver(1).solve(H,g,l,u,l,&a,.5);
  check(!s.success&&!s.converged&&s.iterations<=1,"未找到可行解时不能报告成功");
  check(s.message.find("预算")!=std::string::npos,"预算耗尽不能误报数学不可行");
  Eigen::VectorXd wrong=Eigen::VectorXd::Zero(3);
  check(!solver.solve(H,g,l,u,l,&wrong,0).success,"维度错误");
  check(!solver.solve(Eigen::MatrixXd(0,0),z.head(0),z.head(0),z.head(0),z.head(0)).success,"空问题");
  auto bad=H;bad(0,0)=-1;check(!solver.solve(bad,g,l,u,l).success,"非正定H");
  bad=H;bad(0,0)=std::numeric_limits<double>::quiet_NaN();check(!solver.solve(bad,g,l,u,l).success,"NaN H");
  auto nan=g;nan[0]=std::numeric_limits<double>::infinity();check(!solver.solve(H,nan,l,u,l).success,"Inf g");
  check(!solver.solve(H,g,u,l,l).success,"上下界冲突");
  check(!BoxQPSolver(0).solve(H,g,l,u,l).success,"零预算配置");
  check(!BoxQPSolver(100,0).solve(H,g,l,u,l).success,"零容差配置");
}

// 对原始问题独立检查，不使用求解器自己的残差值。
// 盒端点的符号条件与半空间非负乘子构成KKT证书；固定变量不限制驻点符号。
bool independentKKT(const Eigen::MatrixXd& H,const Eigen::VectorXd& g,
                    const Eigen::VectorXd& l,const Eigen::VectorXd& u,
                    const Eigen::VectorXd& raw_a,double raw_b,const Eigen::VectorXd& x) {
  const double scale=raw_a.cwiseAbs().maxCoeff();
  if(scale==0) return false; // 随机参考集要求非零法向，零法向由解析测试覆盖。
  Eigen::VectorXd a=raw_a/scale;double b=raw_b/scale;const double norm=a.norm();a/=norm;b/=norm;
  const Eigen::VectorXd grad=H*x+g;double mu=0;
  if(std::abs(a.dot(x)-b)<=1e-7) {
    double num=0,den=0,low=0;
    for(Eigen::Index i=0;i<x.size();++i) {
      const bool lo=x[i]<=l[i]+1e-8,hi=x[i]>=u[i]-1e-8;
      if(l[i]==u[i])continue;
      if(!lo&&!hi){num+=a[i]*grad[i];den+=a[i]*a[i];}
      else if((lo&&a[i]<0)||(hi&&a[i]>0))low=std::max(low,grad[i]/a[i]);
    }
    mu=den>1e-16?std::max(0.,num/den):low;
  }
  const Eigen::VectorXd lag=grad-mu*a;
  double residual=0;
  for(Eigen::Index i=0;i<x.size();++i) {
    double v=lag[i];if(l[i]==u[i])v=0;
    else if(x[i]<=l[i]+1e-8)v=std::min(0.,v);
    else if(x[i]>=u[i]-1e-8)v=std::max(0.,v);
    residual=std::max(residual,std::abs(v));
  }
  const double objective=.5*x.dot(H*x)+g.dot(x);
  return residual<=1e-6*(1+grad.norm()+mu*a.norm()) &&
         std::abs(mu*(a.dot(x)-b))<=1e-6*(1+std::abs(objective));
}
}
int main(int argc,char** argv) {
  try {
    selfTests();
    if(argc<2||argc>3) {std::cerr<<"用法：test_box_qp_standalone 用例文件 [期望数量，默认2000]\n";return 2;}
    std::size_t consumed=0;int expected=argc==3?std::stoi(argv[2],&consumed):2000;
    if(expected<=0||(argc==3&&consumed!=std::string(argv[2]).size()))throw std::runtime_error("期望数量无效");
    std::ifstream in(argv[1]);if(!in)throw std::runtime_error("无法打开QP用例文件");
    int total=0,bad=0,not_converged=0;double max_gap=0,total_us=0,max_us=0;
    std::vector<double> times;std::string line;BoxQPSolver solver(100,1e-7);
    while(std::getline(in,line)) {
      if(line.empty())continue;
      std::istringstream row(line);int n=0;
      if(!(row>>n)||n<=0||n>64)throw std::runtime_error("用例维度无效");
      Eigen::MatrixXd H(n,n);Eigen::VectorXd g(n),l(n),u(n),a(n);double b=0,ref=0;
      for(int i=0;i<n*n;++i)row>>H(i/n,i%n);
      for(auto* v:{&g,&l,&u,&a})for(int i=0;i<n;++i)row>>(*v)[i];
      row>>b>>ref;
      if(!row||!H.allFinite()||!g.allFinite()||!l.allFinite()||!u.allFinite()||
         !a.allFinite()||!std::isfinite(b)||!std::isfinite(ref))throw std::runtime_error("截断或非有限用例");
      std::string extra;if(row>>extra)throw std::runtime_error("用例有多余字段");
      const auto start=std::chrono::steady_clock::now();
      const auto s=solver.solve(H,g,l,u,Eigen::VectorXd::Zero(n),&a,b);
      const double us=std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-start).count();
      total_us+=us;max_us=std::max(max_us,us);times.push_back(us);++total;
      if(!s.converged)++not_converged;
      bool ok=s.success&&s.converged&&s.iterations<=100&&s.x.size()==n&&s.x.allFinite();
      if(ok) {
        const auto& x=s.x;const double gap=std::abs(.5*x.dot(H*x)+g.dot(x)-ref);
        max_gap=std::max(max_gap,gap);
        ok=std::isfinite(gap)&&(x.array()>=l.array()-1e-8).all()&&(x.array()<=u.array()+1e-8).all()&&
           a.dot(x)>=b-1e-8*a.norm()&&gap<=1e-7*std::max(1.,std::abs(ref))&&independentKKT(H,g,l,u,a,b,x);
      }
      if(!ok) {++bad;if(bad<=10)std::cerr<<"失败用例 "<<total<<"："<<s.message<<"\n";}
    }
    if(in.bad()||total!=expected)throw std::runtime_error("用例数量不符或读取失败："+std::to_string(total));
    std::sort(times.begin(),times.end());
    std::cout<<"解析断言 "<<checks<<"；随机用例 "<<total<<"；失败 "<<bad<<"；未收敛 "<<not_converged
             <<"；最大绝对目标差 "<<max_gap<<"\n";
    std::cout<<"求解耗时us：平均 "<<total_us/total<<"，p99 "<<times[static_cast<std::size_t>(.99*(total-1))]
             <<"，最大 "<<max_us<<"\n";
    return bad?1:0;
  } catch(const std::exception& e) {std::cerr<<"测试失败："<<e.what()<<"\n";return 2;}
}
