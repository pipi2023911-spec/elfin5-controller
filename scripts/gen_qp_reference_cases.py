#!/usr/bin/env python3
"""开发机生成可行合成QP参考集。Python3+NumPy+SciPy，不连接ROS或机械臂。
参考解通过SLSQP、原始约束与非负乘子KKT检查。默认拒绝覆盖旧文件。
"""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
import scipy
from scipy.optimize import minimize, nnls

def certify(H,g,l,u,a,b,x):
    if not np.isfinite(x).all() or np.any(x<l-1e-9) or np.any(x>u+1e-9) or a@x<b-1e-9:
        return False
    grad=H@x+g; columns=[]; eye=np.eye(len(x))
    if abs(a@x-b)<1e-8: columns.append(a)
    for i in range(len(x)):
        if abs(x[i]-l[i])<1e-8: columns.append(eye[i])
        if abs(x[i]-u[i])<1e-8: columns.append(-eye[i])
    if not columns: return np.linalg.norm(grad,np.inf)<=1e-7
    _,residual=nnls(np.column_stack(columns),grad)
    return residual<=1e-7*(1+np.linalg.norm(grad))

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--count',type=int,default=2000)
    p.add_argument('--seed',type=int,default=43)
    p.add_argument('--output',type=Path,required=True)
    args=p.parse_args(); meta=args.output.with_suffix(args.output.suffix+'.meta.json')
    if args.count<=0: p.error('数量必须为正')
    if args.output.exists() or meta.exists(): p.error('输出已存在，请使用新文件')
    rng=np.random.default_rng(args.seed)
    rows=[];counts=[0,0,0];attempts=0;rejected=0;n=6
    while len(rows)<args.count:
        attempts+=1
        if attempts>args.count*50: raise RuntimeError('参考求解拒收过多，停止而非无限重试')
        kind=len(rows)%3
        J=rng.normal(scale=1 if kind==0 else .3,size=(3,n))
        H=J.T@J+(1e-2 if kind==0 else 1e-4)*np.eye(n)
        if kind==2:
            v=rng.normal(size=3);v*=.15/np.linalg.norm(v)
            g=-J.T@v-1e-4*rng.normal(size=n)
            prev=rng.uniform(-.3,.3,n);step=.7*12.56*.02
            l=np.maximum(-1.1,prev-step);u=np.minimum(1.1,prev+step)
        else:
            g=rng.normal(size=n);l=-rng.uniform(.05,1,n);u=rng.uniform(.05,1,n)
        a=rng.normal(size=n);b=a@rng.uniform(l,u)
        r=minimize(lambda x:.5*x@H@x+g@x,np.clip(np.zeros(n),l,u),
                   jac=lambda x:H@x+g,bounds=list(zip(l,u)),
                   constraints=[{'type':'ineq','fun':lambda x:a@x-b,'jac':lambda x:a}],
                   method='SLSQP',options={'ftol':1e-14,'maxiter':2000})
        if not r.success or not certify(H,g,l,u,a,b,r.x):
            rejected+=1;continue
        ref=.5*r.x@H@r.x+g@r.x
        rows.append(str(n)+' '+' '.join(format(float(v),'.17g') for v in
                    [*H.ravel(),*g,*l,*u,*a,b,ref])+'\n')
        counts[kind]+=1
    args.output.parent.mkdir(parents=True,exist_ok=True);text=''.join(rows)
    with args.output.open('x',encoding='utf-8',newline='\n') as out:out.write(text)
    report={'seed':args.seed,'count':len(rows),'groups':counts,'attempts':attempts,
            'rejected_reference_solves':rejected,'numpy':np.__version__,'scipy':scipy.__version__,
            'reference':'SLSQP plus independent primal/KKT check',
            'sha256':hashlib.sha256(text.encode()).hexdigest(),
            'scope':'synthetic feasible QPs, not measured robot failure rates'}
    with meta.open('x',encoding='utf-8') as out:json.dump(report,out,indent=2)
    print(json.dumps(report,indent=2))
if __name__=='__main__':main()
