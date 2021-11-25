#pragma once

#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <arpa/inet.h>
#include <ifaddrs.h>

#include <iostream>
#include <sstream>
#include <boost/format.hpp>
#include <thread>
#include <chrono>
#include <mutex>

#include <zmq.hpp>

#include <ros/ros.h>

class ZMQ_CLASS{
public:
  explicit ZMQ_CLASS(ros::NodeHandle nh);
  ~ZMQ_CLASS();
  
  std::string getIPAddress();

  std::string zipcode_;
  std::string rad_group_, dsh_group_;
  std::string udp_ip_, tcpsub_ip_, tcppub_ip_, tcpreq_ip_, tcprep_ip_;

  bool controlDone_;
  std::string send_req_, recv_req_, send_rep_, recv_rep_, recv_sub_, send_pub_, send_rad_, recv_dsh_;
  
private:
  ros::NodeHandle nodeHandle_;
  void init();
  bool readParameters();
  void* subscribeZMQ();
  void* publishZMQ();
  void* requestZMQ();
  void* replyZMQ();
  void* radioZMQ();
  void* dishZMQ();
  
  std::string interface_name_;
  std::thread subThread_, pubThread_, reqThread_, repThread_, radThread_, dshThread_;
  zmq::context_t context_;
  zmq::socket_t sub_socket_, pub_socket_, req_socket_, rep_socket_, rad_socket_, dsh_socket_;
  bool sub_flag_, pub_flag_, rad_flag_, dsh_flag_, req_flag_, rep_flag_;
};
