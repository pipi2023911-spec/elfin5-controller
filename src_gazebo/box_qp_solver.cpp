/*
 * 正定盒约束 + 单半空间QP：盒有效集，半空间乘子有界搜索。
 * 所有收敛声明都由原问题的可行性、驻点/对偶与互补残差确认。
 * max_iterations 是本次solve的总有效集步数预算，不是每个子问题独享预算。
 */
#include "elfin5_qp_controller/box_qp_solver.h"
#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace elfin5_qp_controller {
namespace {
struct BoxResult {
  Eigen::VectorXd x;
  std::vector<int> state;
  bool optimal=false, numerical_failure=false;
};

// 在可行盒内沿自由子空间最优方向前进；固定变量l=u不参与释放。
BoxResult solveBox(const Eigen::MatrixXd& H, const Eigen::VectorXd& g,
                   const Eigen::VectorXd& l, const Eigen::VectorXd& u,
                   const Eigen::VectorXd& initial, int& remaining, int& used) {
  BoxResult r; const Eigen::Index n=g.size();
  r.x=initial.cwiseMax(l).cwiseMin(u); r.state.assign(n,0);
  for(Eigen::Index i=0;i<n;++i) {
    if(l[i]==u[i]) r.state[i]=2;
    else if(r.x[i]==l[i]) r.state[i]=-1;
    else if(r.x[i]==u[i]) r.state[i]=1;
  }
  while(remaining>0) {
    --remaining; ++used;
    std::vector<Eigen::Index> free;
    for(Eigen::Index i=0;i<n;++i) if(r.state[i]==0) free.push_back(i);
    Eigen::VectorXd target=r.x;
    if(!free.empty()) {
      const Eigen::Index nf=free.size();
      Eigen::MatrixXd A(nf,nf); Eigen::VectorXd rhs(nf);
      for(Eigen::Index k=0;k<nf;++k) {
        const auto i=free[k]; rhs[k]=-g[i];
        for(Eigen::Index j=0;j<n;++j) if(r.state[j]!=0) rhs[k]-=H(i,j)*r.x[j];
        for(Eigen::Index j=0;j<nf;++j) A(k,j)=H(i,free[j]);
      }
      Eigen::LLT<Eigen::MatrixXd> factor(A);
      if(factor.info()!=Eigen::Success) { r.numerical_failure=true; return r; }
      const Eigen::VectorXd value=factor.solve(rhs);
      if(factor.info()!=Eigen::Success || !value.allFinite()) { r.numerical_failure=true; return r; }
      for(Eigen::Index k=0;k<nf;++k) target[free[k]]=value[k];
    }
    const Eigen::VectorXd direction=target-r.x;
    double alpha=1; Eigen::Index blocking=-1; int side=0;
    for(const auto i:free) {
      if(direction[i]>0 && target[i]>u[i]) {
        const double step=(u[i]-r.x[i])/direction[i];
        if(step<alpha) {alpha=step;blocking=i;side=1;}
      } else if(direction[i]<0 && target[i]<l[i]) {
        const double step=(l[i]-r.x[i])/direction[i];
        if(step<alpha) {alpha=step;blocking=i;side=-1;}
      }
    }
    r.x+=alpha*direction; r.x=r.x.cwiseMax(l).cwiseMin(u);
    if(!r.x.allFinite()) {r.numerical_failure=true;return r;}
    if(blocking>=0) {
      r.state[blocking]=side; r.x[blocking]=side<0?l[blocking]:u[blocking]; continue;
    }
    const Eigen::VectorXd grad=H*r.x+g;
    if(!grad.allFinite()) {r.numerical_failure=true;return r;}
    Eigen::Index release=-1;
    double worst=64*std::numeric_limits<double>::epsilon()*
                 (1+(H*r.x).lpNorm<Eigen::Infinity>()+g.lpNorm<Eigen::Infinity>());
    for(Eigen::Index i=0;i<n;++i) {
      const double violation=r.state[i]==-1?-grad[i]:r.state[i]==1?grad[i]:0;
      if(violation>worst) {worst=violation;release=i;}
    }
    if(release<0) {r.optimal=true;return r;}
    r.state[release]=0;
  }
  return r;
}

struct Certificate {double primal=0, kkt=0;};
Certificate certificate(const Eigen::MatrixXd& H,const Eigen::VectorXd& g,
                        const Eigen::VectorXd& l,const Eigen::VectorXd& u,
                        const Eigen::VectorXd& x,const Eigen::VectorXd* a,double b,double mu) {
  Certificate c;
  if(x.size()!=g.size() || !x.allFinite() || !std::isfinite(mu)) {
    c.primal=c.kkt=std::numeric_limits<double>::infinity();return c;
  }
  const Eigen::VectorXd hx=H*x;
  const Eigen::VectorXd grad=hx+g;
  const Eigen::VectorXd lag=a?Eigen::VectorXd(grad-mu*(*a)):grad;
  if(!hx.allFinite()||!lag.allFinite()) {
    c.primal=c.kkt=std::numeric_limits<double>::infinity();return c;
  }
  double stationarity=0;
  for(Eigen::Index i=0;i<x.size();++i) {
    const double scale=1+std::max(std::abs(l[i]),std::abs(u[i]));
    c.primal=std::max(c.primal,std::max(l[i]-x[i],x[i]-u[i])/scale);
    double v=lag[i];
    if(l[i]==u[i]) v=0;
    else if(x[i]<=l[i]) v=std::min(0.0,v);
    else if(x[i]>=u[i]) v=std::max(0.0,v);
    stationarity=std::max(stationarity,std::abs(v));
  }
  const double dual_scale=1+hx.lpNorm<Eigen::Infinity>()+g.lpNorm<Eigen::Infinity>()+
                          (a?std::abs(mu)*a->lpNorm<Eigen::Infinity>():0);
  if(!std::isfinite(dual_scale)) {c.primal=c.kkt=std::numeric_limits<double>::infinity();return c;}
  c.kkt=stationarity/dual_scale;
  if(a) {
    const double slack=a->dot(x)-b;
    c.primal=std::max(c.primal,-slack/(1+std::abs(b)));
    const double objective_scale=1+std::abs(.5*x.dot(hx))+std::abs(g.dot(x));
    if(!std::isfinite(slack)||!std::isfinite(objective_scale)||!std::isfinite(mu*slack)) {
      c.primal=c.kkt=std::numeric_limits<double>::infinity();return c;
    }
    c.kkt=std::max(c.kkt,std::abs(mu*slack)/objective_scale);
    c.kkt=std::max(c.kkt,std::max(0.0,-mu)/(1+std::abs(mu)));
  }
  c.kkt=std::max(c.kkt,c.primal);return c;
}

// 固定本次有效集的乘子导数。只作为区间内牛顿候选，不作为最终正确性证明。
double slope(const Eigen::MatrixXd& H,const Eigen::VectorXd& a,const BoxResult& r) {
  std::vector<Eigen::Index> free;
  for(Eigen::Index i=0;i<a.size();++i) if(r.state[i]==0) free.push_back(i);
  if(free.empty()) return 0;
  Eigen::MatrixXd A(free.size(),free.size()); Eigen::VectorXd af(free.size());
  for(std::size_t i=0;i<free.size();++i) {
    af[i]=a[free[i]];
    for(std::size_t j=0;j<free.size();++j) A(i,j)=H(free[i],free[j]);
  }
  Eigen::LLT<Eigen::MatrixXd> factor(A);
  if(factor.info()!=Eigen::Success) return 0;
  const Eigen::VectorXd direction=factor.solve(af);
  const double value=af.dot(direction);
  return direction.allFinite() && std::isfinite(value) && value>0?value:0;
}
} // namespace

BoxQPSolver::BoxQPSolver(int max_iterations,double tolerance)
 :max_iterations_(max_iterations),tolerance_(tolerance) {}

QPSolution BoxQPSolver::solve(const Eigen::MatrixXd& hessian,const Eigen::VectorXd& g,
 const Eigen::VectorXd& l,const Eigen::VectorXd& u,const Eigen::VectorXd& initial,
 const Eigen::VectorXd* normal,double offset) const {
  QPSolution result;
  result.projected_gradient_norm=std::numeric_limits<double>::infinity();
  const Eigen::Index n=g.size();
  if(n<=0 || hessian.rows()!=n || hessian.cols()!=n || l.size()!=n || u.size()!=n || initial.size()!=n ||
     (normal && normal->size()!=n)) {result.message="QP 矩阵或向量维度不一致或为空";return result;}
  if(!hessian.allFinite()||!g.allFinite()||!l.allFinite()||!u.allFinite()||!initial.allFinite()||
     !std::isfinite(offset)||(normal&&!normal->allFinite())) {result.message="QP 输入包含非有限数值";return result;}
  if(max_iterations_<=0 || !std::isfinite(tolerance_) || tolerance_<=0) {result.message="QP 求解器配置无效";return result;}
  if((l.array()>u.array()).any()) {result.message="QP 上下界不可行";return result;}
  const Eigen::MatrixXd H=.5*hessian+.5*hessian.transpose();
  if(!H.allFinite()) {result.message="QP Hessian 数值溢出";return result;}
  Eigen::LLT<Eigen::MatrixXd> positive(H);
  if(positive.info()!=Eigen::Success) {result.message="QP Hessian 矩阵不是正定矩阵";return result;}

  Eigen::VectorXd a; double b=0; bool use=false;
  if(normal) {
    const double scale=normal->cwiseAbs().maxCoeff();
    if(scale==0) {
      if(offset>0) {result.message="零法向半空间不可行";return result;}
    } else {
      a=(*normal)/scale; const double norm=a.norm();
      b=(offset/scale)/norm; a/=norm;
      if(!std::isfinite(b)) {result.message="半空间归一化数值溢出";return result;}
      use=true;
      long double maximum=0;
      for(Eigen::Index i=0;i<n;++i) maximum+=static_cast<long double>(a[i])*(a[i]>0?u[i]:l[i]);
      const long double roundoff=64*std::numeric_limits<double>::epsilon()*(1+std::abs(maximum));
      if(maximum+roundoff<static_cast<long double>(b)) {result.message="奇异性半空间与关节上下界无法同时满足";return result;}
      if(maximum<static_cast<long double>(b)) b=static_cast<double>(maximum);
    }
  }
  int remaining=max_iterations_;
  BoxResult box=solveBox(H,g,l,u,initial,remaining,result.iterations);
  double mu=0;
  auto finish=[&](const BoxResult& r,double multiplier,const std::string& fallback) {
    result.x=r.x;
    const auto c=certificate(H,g,l,u,r.x,use?&a:nullptr,b,multiplier);
    result.projected_gradient_norm=c.kkt;
    result.success=!r.numerical_failure && std::isfinite(c.kkt) && c.primal<=tolerance_;
    result.converged=result.success && r.optimal && c.kkt<=tolerance_;
    result.message=result.converged?"QP 已收敛":result.success?
      "QP 结果可行但未满足完整KKT容差（预算或数值精度限制）":fallback;
  };
  if(box.numerical_failure) {finish(box,0,"QP 线性求解发生数值失败");return result;}
  if(!use || a.dot(box.x)>=b) {finish(box,0,"QP 未获得满足约束的结果");return result;}
  if(!box.optimal || remaining==0) {finish(box,0,"QP 迭代预算耗尽，未找到可行解");return result;}

  double low=0,high=1;
  BoxResult low_box=box, high_box=box;
  bool bracket=false;
  // 所有有效集步骤共用remaining；没有隐含的每次100步子预算。
  while(remaining>0) {
    high_box=solveBox(H,g-high*a,l,u,low_box.x,remaining,result.iterations);
    if(high_box.numerical_failure) {finish(high_box,high,"QP 乘子子问题数值失败");return result;}
    if(!high_box.optimal) {finish(high_box,high,"QP 迭代预算耗尽，未找到可行解");return result;}
    if(a.dot(high_box.x)>=b) {bracket=true;break;}
    low=high;low_box=high_box;
    if(high>std::numeric_limits<double>::max()/4) break;
    high*=4;
  }
  if(!bracket) {finish(high_box,high,"QP 乘子搜索预算耗尽或数值范围不足");return result;}
  mu=high;
  while(remaining>0) {
    const auto cert=certificate(H,g,l,u,high_box.x,&a,b,high);
    // 根精度比公开验收容差更紧，避免大目标项相消时仅相对残差合格却目标差偏大。
    const double root_tolerance=std::min(tolerance_,1e-11);
    if(cert.kkt<=root_tolerance && std::abs(a.dot(high_box.x)-b)<=root_tolerance*(1+std::abs(b))) break;
    // 使用区间端点的真实有效集计算斜率；不复用其他试探点的state。
    double candidate=.5*low+.5*high;
    const double derivative=slope(H,a,high_box);
    if(derivative>0) {
      const double next=high-(a.dot(high_box.x)-b)/derivative;
      if(std::isfinite(next) && next>low && next<high) candidate=next;
    }
    if(!(candidate>low && candidate<high)) break;
    BoxResult next=solveBox(H,g-candidate*a,l,u,high_box.x,remaining,result.iterations);
    if(next.numerical_failure) {finish(next,candidate,"QP 乘子子问题数值失败");return result;}
    if(!next.optimal) break; // 返回已知可行的high，不用未求完子问题更新区间。
    if(a.dot(next.x)>=b) {high=candidate;high_box=next;}
    else {low=candidate;low_box=next;}
  }
  mu=high;finish(high_box,mu,"QP 未获得有效的可行解");
  return result;
}
} // namespace elfin5_qp_controller
