#include "plotter.hpp"

#include <arpa/inet.h>   // htons, inet_addr
#include <sys/socket.h>  // socket, sendto
#include <unistd.h>      // close

namespace tools
{
Plotter::Plotter(std::string host, uint16_t port)
{
  socket_ = ::socket(AF_INET, SOCK_DGRAM, 0);//Pv4 网络，UDP 协议

  destination_.sin_family = AF_INET;//告诉操作系统这个地址是什么类型
  destination_.sin_port = ::htons(port);//主机字节序 → 网络大端字节序
  destination_.sin_addr.s_addr = ::inet_addr(host.c_str());//转为系统需要的二进制 IP 地址
}

Plotter::~Plotter() { ::close(socket_); }

void Plotter::plot(const nlohmann::json & json)
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto data = json.dump();//把 json 对象序列化为 std::string 字符串
  ::sendto(//UDP 发送函数，把字符串数据包发给指定 IP、端口的上位机
    socket_, data.c_str(), data.length(), 0, reinterpret_cast<sockaddr *>(&destination_),
    sizeof(destination_));
}

}  // namespace tools