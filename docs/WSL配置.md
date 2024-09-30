# WSL 配置

>Rvosuke
>
>1/10/2024 :two: :zero: :cake: :birthday:

cpp项目的跨平台开发比较复杂，本项目原始在windows进行开发，使用Visio Studio的编译器，其配置复杂度不言而喻。但是使用Cmake可以辅助跨平台的开发，在windows上同样可以使用wsl进行开发，而wsl的配置不在赘述。

## Cmake 安装

原始的wsl中是不会自带cmake安装的，只需要在终端中键入`sudo apt install cmake`即可完成安装。

## PCL安装

pcl是比较著名的，我们可以直接使用apt进行安装`sudo apt intall libpcl-dev`，pcl库比较大，不妨先坐下来喝个茶水，吃个蛋糕。之后cmake会自动检索我们在cmake中的配置。

## CLion配置

CLion作为优质的Jetbrian系列产品，自然包含了Remote的开发设置。我们无需在wsl中额外安装CLion，直接使用Windows中安装的CLion即可，在`settings->Toolchains`中添加并配置默认的编译器为wsl（添加wsl时候会自动检测系统的，将其上移到第一位即可设置为默认），之后等待一会，CLion即可配置成功了。

