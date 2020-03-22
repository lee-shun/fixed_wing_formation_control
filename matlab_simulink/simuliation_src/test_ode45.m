%本程序用来测试RK45求解变系数微分方程，参考matlab中文文档
global a;


dt=0.02;
a=1.0;
y=zeros(1,150);%变量输出
time_stamp=0;

for time=0.0:dt:5.0
    
    time_stamp=time_stamp+1;
    
    y(time_stamp+1)=RK45(time,y(time_stamp),f,dt,time_stamp)
    
end