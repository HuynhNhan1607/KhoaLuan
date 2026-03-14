#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

/* Fix #3: aruco headers từ vendor/include/ (tải bằng make deps) */
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>

/* Fix #2: shared constants với mini_server */
#include "camera_docking.h"

namespace
{
/* Fix #2: dùng constants từ camera_docking.h thay vì hardcode */
constexpr int kServerPort    = CAMERA_DOCKING_PORT;
constexpr int kCameraIndex   = CAMERA_DOCKING_INDEX;
constexpr int kFrameWidth    = CAMERA_DOCKING_WIDTH;
constexpr int kFrameHeight   = CAMERA_DOCKING_HEIGHT;
constexpr float kMarkerSizeMeter = CAMERA_DOCKING_MARKER_SIZE;

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
  /* Fix: cv::FileStorage KHÔNG đọc được NumPy .npz (binary format).
   * calibrate.py bây giờ export thêm calib_data.yaml → dùng file này.
   * Thứ tự thử: YAML từ thư mục camera/ (khi chạy từ mini_server/),
   *             YAML local (khi chạy trực tiếp từ camera/),
   *             sau đó fallback intrinsics.
   */
  const char *yaml_paths[] = {"camera/calib_data.yaml", "calib_data.yaml", nullptr};
  for (int i = 0; yaml_paths[i] != nullptr; ++i)
  {
    cv::FileStorage fs(yaml_paths[i], cv::FileStorage::READ);
    if (!fs.isOpened())
    {
      continue;
    }
    fs["mtx"] >> camera_matrix;
    fs["dist"] >> dist_coeffs;
    fs.release();
    if (!camera_matrix.empty() && !dist_coeffs.empty())
    {
      std::printf("[VISION] Loaded calibration from %s\n", yaml_paths[i]);
      return true;
    }
  }

  std::fprintf(stderr,
               "[VISION] Calibration load failed (tried calib_data.yaml). "
               "Run camera/calibrate.py to generate it. Using fallback intrinsics.\n");
  // Fallback camera intrinsics (rough default for 640x480 stream)
  camera_matrix = (cv::Mat_<double>(3, 3) << 600.0, 0.0, 320.0, 0.0, 600.0,
                   240.0, 0.0, 0.0, 1.0);
  dist_coeffs = cv::Mat::zeros(1, 5, CV_64F);
  return true;
}

/* ============================================================
 * detect_color_hint: phát hiện màu hộp, so sánh số pixel
 * bên trái vs phải → trả về "LEFT", "RIGHT", "CENTER", "NONE"
 *
 * Màu vàng chuối (HSV OpenCV, H range 0-180):
 *   H: 15-35  S: 80-255  V: 80-255
 * Có thể override qua env: YELLOW_H_LO, YELLOW_H_HI, YELLOW_S_LO, YELLOW_V_LO
 * ============================================================ */
const char *detect_color_hint(const cv::Mat &frame)
{
  static int h_lo = -1, h_hi, s_lo, v_lo;
  if (h_lo < 0)
  {
    auto get_env_int = [](const char *name, int def) {
      const char *v = std::getenv(name);
      return v ? std::atoi(v) : def;
    };
    h_lo = get_env_int("YELLOW_H_LO", 15);
    h_hi = get_env_int("YELLOW_H_HI", 35);
    s_lo = get_env_int("YELLOW_S_LO", 80);
    v_lo = get_env_int("YELLOW_V_LO", 80);
    std::printf("[VISION] Color hint HSV range H:[%d,%d] S>=%d V>=%d\n",
                h_lo, h_hi, s_lo, v_lo);
  }

  if (frame.empty()) return "NONE";

  cv::Mat hsv;
  cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

  cv::Mat mask;
  cv::inRange(hsv,
              cv::Scalar(h_lo, s_lo, v_lo),
              cv::Scalar(h_hi, 255,   255),
              mask);

  // Tổng số pixel vàng bên trái và phải
  int mid = frame.cols / 2;
  int left_count  = cv::countNonZero(mask(cv::Rect(0,   0, mid,            frame.rows)));
  int right_count = cv::countNonZero(mask(cv::Rect(mid, 0, frame.cols-mid, frame.rows)));
  int total = left_count + right_count;

  // Ngưỡng tối thiểu để không bị noise (0.5% diện tích frame)
  int min_pixels = (frame.rows * frame.cols) / 200;
  if (total < min_pixels) return "NONE";

  float ratio = static_cast<float>(left_count) / static_cast<float>(total);
  // ratio > 0.6 → phần lớn màu bên trái → strafe LEFT
  // ratio < 0.4 → phần lớn màu bên phải → strafe RIGHT
  // còn lại → CENTER
  if      (ratio > 0.6f) return "LEFT";
  else if (ratio < 0.4f) return "RIGHT";
  else                   return "CENTER";
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

int main(int argc, char *argv[])
{
  // Cho phép override camera index: ./pose_estimate_service [index]
  int cam_index = kCameraIndex;
  if (argc >= 2)
  {
    cam_index = std::atoi(argv[1]);
    std::printf("[VISION] Using camera index %d (from argv)\n", cam_index);
  }

  cv::Mat camera_matrix, dist_coeffs;
  if (!load_calibration(camera_matrix, dist_coeffs))
  {
    return 1;
  }

  // Thử cam_index trước, nếu fail thử fallback 0 và 1
  cv::VideoCapture cap;
  std::vector<int> indices_to_try = {cam_index};
  if (cam_index != 0) indices_to_try.push_back(0);
  if (cam_index != 1) indices_to_try.push_back(1);

  for (int idx : indices_to_try)
  {
    cap.open(idx);
    if (cap.isOpened())
    {
      if (idx != cam_index)
        std::printf("[VISION] Camera index %d failed, using index %d instead\n", cam_index, idx);
      cam_index = idx;
      break;
    }
    cap.release();
  }

  if (!cap.isOpened())
  {
    std::fprintf(stderr, "[VISION] Failed to open any camera (tried:");
    for (int idx : indices_to_try) std::fprintf(stderr, " %d", idx);
    std::fprintf(stderr, ")\n");
    std::fprintf(stderr, "[VISION] Hint: check 'ls /dev/video*' and pass index as arg: ./pose_estimate_service 0\n");
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

  /* Fix #5: Old ArUco API — compatible với OpenCV 4.2 / 4.5.4 (JetPack)
   *   ArucoDetector class chỉ có từ OpenCV 4.7+, không dùng được trên JetPack.
   *   Dùng cv::Ptr<Dictionary> + free function cv::aruco::detectMarkers() thay thế.
   */
  cv::Ptr<cv::aruco::Dictionary> aruco_dict =
      cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
  cv::Ptr<cv::aruco::DetectorParameters> detector_params =
      cv::aruco::DetectorParameters::create();

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
      usleep(CAMERA_DOCKING_LOOP_DELAY_US);
      continue;
    }

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    /* Fix #5: free function thay cho detector.detectMarkers() */
    cv::aruco::detectMarkers(frame, aruco_dict, corners, ids, detector_params);

    int found = 0;
    float x_meter = 0.0f;   // trái(-) / phải(+) so với camera
    float y_meter = 0.0f;   // xuống(-) / lên(+) so với camera
    float z_meter = 0.0f;   // khoảng cách (depth)

    if (!ids.empty())
    {
      for (size_t i = 0; i < ids.size(); ++i)
      {
        if (ids[i] != CAMERA_DOCKING_MARKER_ID)
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
          y_meter = static_cast<float>(tvecs[0][1]);
          z_meter = static_cast<float>(tvecs[0][2]);
        }
        break;
      }
    }

    // Xác định hướng trái/phải (ngưỡng 2cm tránh jitter)
    const char *direction = "CENTER";
    if      (x_meter < -0.02f) direction = "LEFT";
    else if (x_meter >  0.02f) direction = "RIGHT";

    // Nếu không thấy marker → dùng color hint để guide search
    const char *hint = found ? direction
                             : detect_color_hint(frame);

    char payload[256];
    std::snprintf(payload, sizeof(payload),
                  "{\"type\":\"docking_vision\",\"found\":%d,"
                  "\"x\":%.5f,\"y\":%.5f,\"z\":%.5f,"
                  "\"hint\":\"%s\"}\n",
                  found, x_meter, y_meter, z_meter, hint);
    send_line_if_connected(client_fd, payload);

    // Log khi trạng thái found thay đổi
    if (last_found != found)
    {
      if (found)
        std::printf("[VISION] DETECTED  dir=%-6s x=%+.3fm y=%+.3fm z=%.3fm\n",
                    direction, x_meter, y_meter, z_meter);
      else
        std::printf("[VISION] LOST  color_hint=%s\n", hint);
      last_found = found;
    }

    // Tick log mỗi ~1 giây (20 iterations × 50ms)
    if (++debug_tick >= 20)
    {
      if (found)
        std::printf("[VISION] %-6s  x=%+.3fm y=%+.3fm z=%.3fm  client=%s\n",
                    direction, x_meter, y_meter, z_meter,
                    client_fd >= 0 ? "connected" : "none");
      else
        std::printf("[VISION] no marker  color_hint=%-6s  client=%s\n",
                    hint, client_fd >= 0 ? "connected" : "none");
      debug_tick = 0;
    }

    // ~20Hz match với vòng điều khiển trajectory
    usleep(CAMERA_DOCKING_LOOP_DELAY_US);
  }

  if (client_fd >= 0)
  {
    close(client_fd);
  }
  close(server_fd);
  return 0;
}
