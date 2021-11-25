#include "zmq_class/zmq_class.h"

ZMQ_CLASS::ZMQ_CLASS(ros::NodeHandle nh)
  :nodeHandle_(nh), context_(1)
{
  if(!readParameters())
  {
    ros::requestShutdown();
  }

  init();
}

ZMQ_CLASS::~ZMQ_CLASS()
{
  std::cout << "Disconnected" << std::endl;
  controlDone_ = true;
  ros::requestShutdown();
  sub_socket_.close();
  pub_socket_.close();
  req_socket_.close();
  rep_socket_.close();
  rad_socket_.close();
  dsh_socket_.close();
  context_.close();
}

void ZMQ_CLASS::init()
{
  controlDone_ = false;

  /* Initialize Subscribe Socket */
  if(sub_flag_)
  {
    sub_socket_ = zmq::socket_t(context_, ZMQ_SUB);
    sub_socket_.connect(tcpsub_ip_);
    const char *filter = zipcode_.c_str();
    sub_socket_.setsockopt(ZMQ_SUBSCRIBE, filter, strlen(filter)); 
  }

  if(pub_flag_)
  {
    pub_socket_ = zmq::socket_t(context_, ZMQ_PUB);
    pub_socket_.connect(tcppub_ip_);
  }

  /* Initialize Request Socket */
  if(req_flag_) {
    req_socket_ = zmq::socket_t(context_, ZMQ_REQ); 
    req_socket_.connect(tcpreq_ip_);
    //req_socket_.setsockopt(ZMQ_REQ_CORRELATE, 1); 
  }

  /* Initialize Udp send(Radio) Socket */
  if(rad_flag_)
  {
    rad_socket_ = zmq::socket_t(context_, ZMQ_RADIO);
    rad_socket_.connect(udp_ip_);
  }

  /* Initialize Udp recv(Dish) Socket */
  if(dsh_flag_)
  {
    dsh_socket_ = zmq::socket_t(context_, ZMQ_DISH);
    dsh_socket_.bind(udp_ip_);
    dsh_socket_.join(dsh_group_.c_str());
  }

  /* Initialize Threads */
  if(sub_flag_)
    subThread_ = std::thread(&ZMQ_CLASS::subscribeZMQ, this);
  if(pub_flag_)
    pubThread_ = std::thread(&ZMQ_CLASS::publishZMQ, this);
  if(req_flag_)
    reqThread_ = std::thread(&ZMQ_CLASS::requestZMQ, this);
  if(rep_flag_)
    repThread_ = std::thread(&ZMQ_CLASS::replyZMQ, this);
  if(rad_flag_)
    radThread_ = std::thread(&ZMQ_CLASS::radioZMQ, this);
  if(dsh_flag_)
    dshThread_ = std::thread(&ZMQ_CLASS::dishZMQ, this);
}

std::string ZMQ_CLASS::getIPAddress(){
  std::string ipAddress="Unable to get IP Address";
  struct ifaddrs *interfaces = NULL;
  struct ifaddrs *temp_addr = NULL;
  int success = 0;
  success = getifaddrs(&interfaces);
  if (success == 0)
  {
    temp_addr = interfaces;
    while(temp_addr != NULL)
    {
      if(temp_addr->ifa_addr->sa_family == AF_INET)
      {
        if(strcmp(temp_addr->ifa_name, interface_name_.c_str())==0)
	{
          ipAddress = inet_ntoa(((struct sockaddr_in*)temp_addr->ifa_addr)->sin_addr);
	}
      }
      temp_addr = temp_addr->ifa_next;
    }
    freeifaddrs(interfaces);
    return ipAddress;
  }
}

bool ZMQ_CLASS::readParameters()
{
  std::string tcp_ip, tcpsub_port, tcppub_port, tcpreq_port, tcprep_port;
  std::string udp_ip, udp_port;
  nodeHandle_.param("tcp_ip/interface_name",interface_name_,std::string("ens33"));

  nodeHandle_.param("tcp_ip/ip_addr",tcp_ip,std::string("tcp://192.168.85.128"));
  nodeHandle_.param("tcp_ip/sub_port",tcpsub_port,std::string("5555"));
  nodeHandle_.param("tcp_ip/pub_port",tcpsub_port,std::string("5555"));
  nodeHandle_.param("tcp_ip/req_port",tcpreq_port,std::string("4444"));
  nodeHandle_.param("tcp_ip/rep_port",tcpreq_port,std::string("4444"));

  nodeHandle_.param("tcp_ip/zipcode",zipcode_,std::string("00001"));

  nodeHandle_.param("udp_ip/ip_addr",udp_ip,std::string("udp://127.0.0.1"));
  nodeHandle_.param("udp_ip/port",udp_port,std::string("9090"));
  nodeHandle_.param("udp_ip/send_group",rad_group_,std::string("FV1"));
  nodeHandle_.param("udp_ip/recv_group",dsh_group_,std::string("LV"));
  
  nodeHandle_.param("socket/rad_flag",rad_flag_,true);
  nodeHandle_.param("socket/dsh_flag",dsh_flag_,true);
  nodeHandle_.param("socket/req_flag",req_flag_,true);
  nodeHandle_.param("socket/rep_flag",rep_flag_,false);
  nodeHandle_.param("socket/sub_flag",sub_flag_,true);
  nodeHandle_.param("socket/pub_flag",pub_flag_,false);

  tcppub_ip_ = tcp_ip;
  tcppub_ip_.append(":");
  tcppub_ip_.append(tcppub_port);
  tcpsub_ip_ = tcp_ip;
  tcpsub_ip_.append(":");
  tcpsub_ip_.append(tcpsub_port);
  tcpreq_ip_ = tcp_ip;
  tcpreq_ip_.append(":");
  tcpreq_ip_.append(tcpreq_port);
  tcprep_ip_ = tcp_ip;
  tcprep_ip_.append(":");
  tcprep_ip_.append(tcprep_port);
  udp_ip_ = udp_ip;
  udp_ip_.append(":");
  udp_ip_.append(udp_port);

  return true;
}

void* ZMQ_CLASS::subscribeZMQ()
{
  while(sub_socket_.connected() && !controlDone_)
  {
    zmq::message_t update;

    sub_socket_.recv(&update, 0);

    recv_sub_ = static_cast<char*>(update.data());
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}
void* ZMQ_CLASS::requestZMQ()
{ 
  int send_data = 0;

  while(req_socket_.connected() && !controlDone_)
  {
    zmq::message_t request(20);

    snprintf((char *) request.data(), 20, "%s %d", send_req_.c_str());
    req_socket_.send(request);

    int zipcode, recv_data;
    zmq::message_t reply;
    bool rc = req_socket_.recv(&reply, 0);

    recv_req_ = static_cast<char*>(reply.data());
    
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}
void* ZMQ_CLASS::radioZMQ()
{
  while(rad_socket_.connected() && !controlDone_)
  {
    zmq::message_t request(20);
    request.set_group(rad_group_.c_str());

    snprintf((char *) request.data(), 20, "%s", send_rad_.c_str());

    rad_socket_.send(request, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}
void* ZMQ_CLASS::dishZMQ()
{
  while(dsh_socket_.connected() && !controlDone_)
  {
    zmq::message_t reply;

    bool rc = dsh_socket_.recv(&reply, 0);

    recv_dsh_ = static_cast<char*>(reply.data());
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

void* ZMQ_CLASS::publishZMQ()
{
  while(pub_socket_.connected() && !controlDone_)
  {
    zmq::message_t publish;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

void* ZMQ_CLASS::replyZMQ()
{
  while(rep_socket_.connected() && !controlDone_)
  {
    zmq::message_t reply;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

