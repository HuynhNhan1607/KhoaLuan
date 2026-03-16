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
// Anti-flicker tuning for noisy camera feeds.
constexpr int kFoundDebounceFrames = 2;
constexpr int kLostDebounceFrames = 5;
constexpr float kPoseEmaAlpha = 0.35f;
constexpr float kCenterXMeterTol = 0.02f;

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
 * Màu vàng chuối nhạt (HSV OpenCV, H range 0-180):
 *   H: 18-45  S: 25-255  V: 60-255
 * Có thể override qua env: YELLOW_H_LO, YELLOW_H_HI, YELLOW_S_LO, YELLOW_V_LO
 * ============================================================ */
const char *detect_color_hint(const cv::Mat &frame)
{
  static int h_lo = -1, h_hi, s_lo, v_lo;
  static double min_area_ratio = 0.007;
  static double deadband_ratio = 0.10;
  if (h_lo < 0)
  {
    auto get_env_int = [](const char *name, int def) {
      const char *v = std::getenv(name);
      return v ? std::atoi(v) : def;
    };
    auto get_env_double = [](const char *name, double def) {
      const char *v = std::getenv(name);
      return v ? std::atof(v) : def;
    };
    h_lo = get_env_int("YELLOW_H_LO", 18);
    h_hi = get_env_int("YELLOW_H_HI", 45);
    s_lo = get_env_int("YELLOW_S_LO", 25);
    v_lo = get_env_int("YELLOW_V_LO", 60);
    min_area_ratio = get_env_double("YELLOW_MIN_AREA_RATIO", 0.007);
    deadband_ratio = get_env_double("YELLOW_DEADBAND_RATIO", 0.10);
    std::printf("[VISION] Color hint HSV range H:[%d,%d] S>=%d V>=%d\n",
                h_lo, h_hi, s_lo, v_lo);
    std::printf("[VISION] Color hint geometry min_area_ratio=%.4f deadband_ratio=%.3f\n",
                min_area_ratio, deadband_ratio);
  }

  if (frame.empty()) return "NONE";

  cv::Mat hsv;
  cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

  cv::Mat mask;
  cv::inRange(hsv,
              cv::Scalar(h_lo, s_lo, v_lo),
              cv::Scalar(h_hi, 255, 255),
              mask);

  // Khử nhiễu trước khi tìm blob vàng lớn nhất.
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
  cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
  cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  if (contours.empty()) return "NONE";

  // Chọn contour lớn nhất để đại diện hộp (tránh nền vàng lốm đốm).
  double best_area = 0.0;
  int best_idx = -1;
  for (size_t i = 0; i < contours.size(); ++i)
  {
    double a = cv::contourArea(contours[i]);
    if (a > best_area)
    {
      best_area = a;
      best_idx = static_cast<int>(i);
    }
  }
  if (best_idx < 0) return "NONE";

  // Ngưỡng diện tích tối thiểu để loại nhiễu nhỏ nhưng vẫn bắt được hộp xa.
  double min_area = static_cast<double>(frame.cols * frame.rows) * min_area_ratio;
  if (best_area < min_area) return "NONE";

  cv::Moments m = cv::moments(contours[best_idx]);
  if (m.m00 <= 1e-6) return "NONE";

  double cx = m.m10 / m.m00;
  double center = static_cast<double>(frame.cols) * 0.5;
  double deadband = static_cast<double>(frame.cols) * deadband_ratio;

  if (cx < (center - deadband)) return "LEFT";
  if (cx > (center + deadband)) return "RIGHT";
  return "CENTER";
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
  detector_params->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
  detector_params->cornerRefinementMaxIterations = 30;
  detector_params->cornerRefinementMinAccuracy = 0.03;
  detector_params->adaptiveThreshWinSizeMin = 5;
  detector_params->adaptiveThreshWinSizeMax = 31;
  detector_params->adaptiveThreshWinSizeStep = 4;
  detector_params->adaptiveThreshConstant = 7.0;
  detector_params->minMarkerPerimeterRate = 0.02;
  detector_params->maxMarkerPerimeterRate = 4.0;
  detector_params->minCornerDistanceRate = 0.03;
  detector_params->minMarkerDistanceRate = 0.03;

  std::printf("[VISION] Running detection loop...\n");
  int debug_tick = 0;
  int last_found = -1;
    int hit_streak = 0;
    int miss_streak = 0;
    bool stable_found = false;
    int hint_score = 0; // -6..+6, âm nghiêng RIGHT, dương nghiêng LEFT
    float x_filtered = 0.0f;
    float y_filtered = 0.0f;
    float z_filtered = 0.0f;
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
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    // Pass 1: detect trên ảnh gray gốc (ổn định hơn khi marker có texture in).
    cv::aruco::detectMarkers(gray, aruco_dict, corners, ids, detector_params);

    // Pass 2 fallback: CLAHE + blur nhẹ để xử lý vùng sáng không đều.
    if (ids.empty())
    {
      cv::Mat clahe_gray;
      cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(2.0, cv::Size(8, 8));
      clahe->apply(gray, clahe_gray);
      cv::GaussianBlur(clahe_gray, clahe_gray, cv::Size(3, 3), 0.0);
      cv::aruco::detectMarkers(clahe_gray, aruco_dict, corners, ids, detector_params);
    }

    int raw_found = 0;
    float raw_x = 0.0f;   // trái(-) / phải(+) so với camera
    float raw_y = 0.0f;   // xuống(-) / lên(+) so với camera
    float raw_z = 0.0f;   // khoảng cách (depth)

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
          raw_found = 1;
          raw_x = static_cast<float>(tvecs[0][0]);
          raw_y = static_cast<float>(tvecs[0][1]);
          raw_z = static_cast<float>(tvecs[0][2]);
        }
        break;
      }
    }

    if (raw_found)
    {
      hit_streak++;
      miss_streak = 0;
      if (hit_streak >= kFoundDebounceFrames)
      {
        stable_found = true;
      }

      if (stable_found && hit_streak == 1)
      {
        x_filtered = raw_x;
        y_filtered = raw_y;
        z_filtered = raw_z;
      }
      else
      {
        x_filtered = (kPoseEmaAlpha * raw_x) + ((1.0f - kPoseEmaAlpha) * x_filtered);
        y_filtered = (kPoseEmaAlpha * raw_y) + ((1.0f - kPoseEmaAlpha) * y_filtered);
        z_filtered = (kPoseEmaAlpha * raw_z) + ((1.0f - kPoseEmaAlpha) * z_filtered);
      }
    }
    else
    {
      miss_streak++;
      hit_streak = 0;
      if (miss_streak >= kLostDebounceFrames)
      {
        stable_found = false;
      }
    }

    int found = stable_found ? 1 : 0;
    float x_meter = found ? x_filtered : 0.0f;
    float y_meter = found ? y_filtered : 0.0f;
    float z_meter = found ? z_filtered : 0.0f;

    // Xác định hướng trái/phải (ngưỡng 2cm tránh jitter)
    const char *direction = "CENTER";
    if      (x_meter < -kCenterXMeterTol) direction = "LEFT";
    else if (x_meter >  kCenterXMeterTol) direction = "RIGHT";

    // Nếu không thấy marker → dùng color hint để guide search, có tích lũy để tránh nhấp nháy.
    const char *hint_raw = found ? direction : detect_color_hint(frame);
    if (!found)
    {
      if (std::strcmp(hint_raw, "LEFT") == 0)
      {
        if (hint_score < 6) hint_score++;
      }
      else if (std::strcmp(hint_raw, "RIGHT") == 0)
      {
        if (hint_score > -6) hint_score--;
      }
      else
      {
        if (hint_score > 0) hint_score--;
        else if (hint_score < 0) hint_score++;
      }
    }
    else
    {
      hint_score = 0;
    }

    const char *hint = hint_raw;
    if (!found)
    {
      if (hint_score >= 2) hint = "LEFT";
      else if (hint_score <= -2) hint = "RIGHT";
      else if (std::strcmp(hint_raw, "NONE") == 0) hint = "NONE";
      else hint = "CENTER";
    }

        // 1) Legacy wire format for old parser (strict sscanf: found/x/z only)
        // 2) Extended format for new parser (adds y + hint)
        // Gửi cả hai để tương thích ngược mà vẫn hỗ trợ hint khi robot đã cập nhật parser.
        char payload_legacy[256];
        std::snprintf(payload_legacy, sizeof(payload_legacy),
          "{\"type\":\"docking_vision\",\"found\":%d,"
          "\"x\":%.5f,\"z\":%.5f}\n",
          found, x_meter, z_meter);
        send_line_if_connected(client_fd, payload_legacy);

        char payload_ext[256];
        std::snprintf(payload_ext, sizeof(payload_ext),
          "{\"type\":\"docking_vision\",\"found\":%d,"
          "\"x\":%.5f,\"y\":%.5f,\"z\":%.5f,"
          "\"hint\":\"%s\"}\n",
          found, x_meter, y_meter, z_meter, hint);
        send_line_if_connected(client_fd, payload_ext);

    // Log khi trạng thái found thay đổi
    if (last_found != found)
    {
      if (found)
        std::printf("[VISION] DETECTED  dir=%-6s x=%+.3fm y=%+.3fm z=%.3fm (debounce=%d)\n",
                    direction, x_meter, y_meter, z_meter, hit_streak);
      else
        std::printf("[VISION] LOST  color_hint=%s (miss=%d)\n", hint, miss_streak);
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
