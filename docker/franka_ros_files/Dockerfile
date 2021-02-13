FROM nginx

# Modify default conf for CORS access and auto file listing
COPY default.conf /etc/nginx/conf.d/default.conf

#install necessary packages
RUN apt-get update && apt-get install wget unzip -q -y

# download and unzip repository to html serving dir
WORKDIR /tmp
RUN wget -O franka_ros.zip https://github.com/frankaemika/franka_ros/archive/0.6.0.zip

WORKDIR /usr/share/nginx/html
RUN unzip /tmp/franka_ros.zip
RUN mv franka_ros-0.6.0/* .