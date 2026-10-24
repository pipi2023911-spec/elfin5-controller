#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""独立测试程序的文件契约：开发机Python3，不依赖ROS/NumPy。"""
import argparse
from pathlib import Path
import subprocess
import tempfile

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('executable',type=Path)
    args=p.parse_args();exe=str(args.executable.resolve())
    # 单变量min .5*x²，0<=x<=1，x>=.5，参考目标值.125。
    valid='1 1 0 0 1 1 0.5 0.125\n'
    with tempfile.TemporaryDirectory(prefix='elfin5_qp_files_') as folder:
        root=Path(folder)
        cases=[('empty','',False),('truncated','1 1 0\n',False),
               ('extra',valid.strip()+' extra\n',False),
               ('nonfinite','1 1 0 0 1 1 0.5 nan\n',False),
               ('dimension','0 1\n',False),('valid',valid,True)]
        count=0
        for name,text,expected in cases:
            path=root/(name+'.txt');path.write_text(text,encoding='utf-8')
            result=subprocess.run([exe,str(path),'1'],stdout=subprocess.PIPE,stderr=subprocess.PIPE)
            if (result.returncode==0)!=expected:raise RuntimeError('文件验收错误：'+name)
            count+=1
        for command in ([exe,str(root/'missing.txt'),'1'],[exe,str(root/'valid.txt'),'2'],
                        [exe,str(root/'valid.txt'),'abc'],[exe]):
            if subprocess.run(command,stdout=subprocess.PIPE,stderr=subprocess.PIPE).returncode==0:
                raise RuntimeError('缺失文件/数量/参数错误被误判通过')
            count+=1
    print('通过 %d 项回归文件契约检查'%count)
if __name__=='__main__':main()
