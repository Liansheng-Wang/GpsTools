# GpsTools

```angular2html
GPS Tools 使用 ROS 做了上层通讯
并且使用了千寻做为网络基站
代码提供了一些GPS报文解析的python例子，并且封装成了ros包
因为运行时需要依赖硬件配置，所以阅读一下代码。代码很简洁
C++ 和 python 的代码是相互独立的，都可以使用
```


## Python Env install
```angular2html
pip install pyserial==3.5
```

## Ros Env install
```angular2html
sudo apt install ros-noetic-serial
```

## RUN
```angular2html
git clone https://github.com/Liansheng-Wang/GpsTools.git
cd GpsTools
catkin_make
```

如果你熟悉ROS的话，可以查看代码运行