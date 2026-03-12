#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <string>

#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>

namespace
{
constexpr int kServerPort = 9091;
constexpr int kCameraIndex = 1;
constexpr int kFrameWidth = 640;
constexpr int kFrameHeight = 480;
constexpr float kMarkerSizeMeter = 0.05f;

int set_nonblock(int fd)
{
  int flags = fcntl(fd, F_GETFL, 0);
  if (flags < 0)
  {
    return -1;
  }
  return fcntl(fd, F_SETFL, flags | O_NONBLOCK);
}

int create_server_socket()
{
  int fd = socket(AF_INET, SOCK_STREAM, 0);
  if (fd < 0)
  {
    std::perror("[VISION] socket");
    return -1;
  }

  int opt = 1;
  if (setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0)
  {
    std::perror("[VISION] setsockopt");
    close(fd);
    return -1;
  }

  sockaddr_in addr {};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(kServerPort);
  addr.sin_addr.s_addr = inet_addr("127.0.0.1");

  if (bind(fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0)
  {
    std::perror("[VISION] bind");
    close(fd);
    return -1;
  }
  if (listen(fd, 1) < 0)
  {
    std::perror("[VISION] listen");
    close(fd);
    return -1;
  }
  if (set_nonblock(fd) < 0)
  {
    std::perror("[VISION] set_nonblock(server)");
    close(fd);
    return -1;
  }

  std::printf("[VISION] TCP server listening at 127.0.0.1:%d\n", kServerPort);
  return fd;
}

int try_accept_client(int server_fd, int current_client_fd)
{
  if (current_client_fd >= 0)
  {
    return current_client_fd;
  }

  sockaddr_in cli_addr {};
  socklen_t cli_len = sizeof(cli_addr);
  int client_fd =
      accept(server_fd, reinterpret_cast<sockaddr *>(&cli_addr), &cli_len);
  if (client_fd < 0)
  {
    if (errno == EAGAIN || errno == EWOULDBLOCK)
    {
      return -1;
    }
    std::perror("[VISION] accept");
    return -1;
  }
  if (set_nonblock(client_fd) < 0)
  {
    std::perror("[VISION] set_nonblock(client)");
    close(client_fd);
    return -1;
  }
  std::printf("[VISION] mini_server connected\n");
  return client_fd;
}

bool load_calibration(cv::Mat &camera_matrix, cv::Mat &dist_coeffs)
{
  cv::FileStorage fs("camera/calib_data.npz", cv::FileStorage::READ);
  if (!fs.isOpened())
  {
    fs.release();
    fs.open("calib_data.npz", cv::FileStorage::READ);
  }
  if (!fs.isOpened())
  {
    std::fprintf(stderr,
                 "[VISION] Cannot open calib_data.npz. Expected in camera/ or cwd.\n");
    return false;
  }

  fs["mtx"] >> camera_matrix;
  fs["dist"] >> dist_coeffs;
  fs.release();

  if (camera_matrix.empty() || dist_coeffs.empty())
  {
    std::fprintf(stderr, "[VISION] Invalid calibration content in npz file.\n");
    return false;
  }
  return true;
}

void send_line_if_connected(int &client_fd, const std::string &line)
{
  if (client_fd < 0)
  {
    return;
  }

  ssize_t sent = send(client_fd, line.c_str(), line.size(), MSG_NOSIGNAL);
  if (sent < 0)
  {
    if (errno != EAGAIN && errno != EWOULDBLOCK)
    {
      std::perror("[VISION] send");
      std::printf("[VISION] mini_server disconnected\n");
      close(client_fd);
      client_fd = -1;
    }
  }
}
} // namespace

int main()
{
  cv::Mat camera_matrix, dist_coeffs;
  if (!load_calibration(camera_matrix, dist_coeffs))
  {
    return 1;
  }

  cv::VideoCapture cap(kCameraIndex);
  if (!cap.isOpened())
  {
    std::fprintf(stderr, "[VISION] Failed to open camera index %d\n",
                 kCameraIndex);
    return 1;
  }
  cap.set(cv::CAP_PROP_FRAME_WIDTH, kFrameWidth);
  cap.set(cv::CAP_PROP_FRAME_HEIGHT, kFrameHeight);

  int server_fd = create_server_socket();
  if (server_fd < 0)
  {
    return 1;
  }
  int client_fd = -1;

  cv::aruco::Dictionary aruco_dict =
      cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
  cv::aruco::DetectorParameters detector_params;
  cv::aruco::ArucoDetector detector(aruco_dict, detector_params);

  std::printf("[VISION] Running detection loop...\n");
  int debug_tick = 0;
  int last_found = -1;
  while (true)
  {
    client_fd = try_accept_client(server_fd, client_fd);

    cv::Mat frame;
    if (!cap.read(frame) || frame.empty())
    {
      std::fprintf(stderr, "[VISION] Empty frame, retrying...\n");
      usleep(50000);
      continue;
    }

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    detector.detectMarkers(frame, corners, ids);

    int found = 0;
    float x_meter = 0.0f;
    float z_meter = 0.0f;

    if (!ids.empty())
    {
      for (size_t i = 0; i < ids.size(); ++i)
      {
        if (ids[i] != 0)
        {
          continue;
        }

        std::vector<cv::Vec3d> rvecs;
        std::vector<cv::Vec3d> tvecs;
        cv::aruco::estimatePoseSingleMarkers(
            std::vector<std::vector<cv::Point2f>>{corners[i]}, kMarkerSizeMeter,
            camera_matrix, dist_coeffs, rvecs, tvecs);
        if (!tvecs.empty())
        {
          found = 1;
          x_meter = static_cast<float>(tvecs[0][0]);
          z_meter = static_cast<float>(tvecs[0][2]);
        }
        break;
      }
    }

    char payload[256];
    std::snprintf(payload, sizeof(payload),
                  "{\"type\":\"docking_vision\",\"found\":%d,\"x\":%.5f,\"z\":%.5f}\n",
                  found, x_meter, z_meter);
    send_line_if_connected(client_fd, payload);

    if (last_found != found)
    {
      std::printf("[VISION] found=%d x=%.4f z=%.4f\n", found, x_meter, z_meter);
      last_found = found;
    }
    if (++debug_tick >= 20)
    {
      std::printf("[VISION] tick found=%d x=%.4f z=%.4f client=%s\n", found,
                  x_meter, z_meter, client_fd >= 0 ? "connected" : "none");
      debug_tick = 0;
    }

    // ~20Hz match với vòng điều khiển trajectory
    usleep(50000);
  }

  if (client_fd >= 0)
  {
    close(client_fd);
  }
  close(server_fd);
  return 0;
}
