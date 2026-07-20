/*
 * imx219_camera_node.cpp
 * ROS 2 Humble camera node for IMX219 on Orange Pi Zero 3W (Allwinner A733)
 *
 * V4L2 capture code is taken verbatim from camera_shm_host.cpp (proven working).
 * Instead of writing to SHM, frames are published directly to ROS 2 topics.
 *
 * Publishes:
 *   <topic>       sensor_msgs/Image
 *   <topic>_info  sensor_msgs/CameraInfo
 *
 * Parameters:
 *   topic            string   /camera_1/image_raw
 *   source_type      string   v4l2          (A733 supports v4l2)
 *   sensor           string   imx219
 *   fps              int      30
 *   width            int      1280
 *   height           int      960
 *   resize_w         int      0             (0 = native)
 *   resize_h         int      0             (0 = native)
 *   video_device     string   /dev/video8
 *   format           string   BGR24
 *   io_mode          string   mmap
 *   qos_reliable     bool     true
 *   calibration_file string   ""
 *   enable_isp       bool     true
 *   frame_id         string   camera_optical_1
 *   flip_180         bool     false         (rotate image 180 deg, for upside-down mounting)
 *
 * Legacy aliases:
 *   device           string   alias for video_device
 *   in_size          string   WIDTHxHEIGHT alias for width/height
 */

#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <pthread.h>
#include <sched.h>
#include <signal.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <cv_bridge/cv_bridge.h>

#ifdef USE_AWI_SP
extern "C" {
#include "AWIspApi.h"
}
#endif

// ── Helpers ───────────────────────────────────────────────────────────────────

static int64_t now_ms() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

static bool parse_size(const std::string& s, int& w, int& h) {
    auto x = s.find('x');
    if (x == std::string::npos) return false;
    try { w = std::stoi(s.substr(0, x)); h = std::stoi(s.substr(x + 1)); }
    catch (...) { return false; }
    return w > 0 && h > 0;
}

static void try_set_realtime(int prio = 10) {
    sched_param sp{}; sp.sched_priority = prio;
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) == 0)
        fprintf(stdout, "[RT] SCHED_FIFO prio=%d OK\n", prio);
    else
        fprintf(stderr, "[RT] SCHED_FIFO failed — need root or CAP_SYS_NICE\n");
}

// ── V4L2 Camera (verbatim from camera_shm_host.cpp) ─────────────────────────

class V4L2Camera {
public:
    static constexpr unsigned BUF_COUNT = 4;

    V4L2Camera(const std::string& device, int width, int height, int fps)
        : device_(device), width_(width), height_(height),
          fd_(-1), streaming_(false), nplanes_(0), fps_(fps)
    {
        video_id_ = parseVideoId(device_);
        for (unsigned i = 0; i < BUF_COUNT; i++)
            for (unsigned j = 0; j < 3; j++) {
                buffers_[i].start[j]  = MAP_FAILED;
                buffers_[i].length[j] = 0;
            }
    }

    ~V4L2Camera() {
        stop();
        releaseBuffers();
        if (fd_ >= 0) close(fd_);
    }

    // Must be called BEFORE init(): the flip changes the sensor's Bayer (CFA)
    // phase, and that must be latched before S_FMT so the ISP demosaics with the
    // correct pattern. Setting it after STREAMON swaps R<->B in the output.
    void requestFlip(bool hflip, bool vflip) {
        want_hflip_ = hflip;
        want_vflip_ = vflip;
    }
    bool sensorFlipOk() const { return sensor_flip_ok_; }

    bool init() {
        if (!openDevice())     { fprintf(stderr,"[V4L2] openDevice failed\n");  return false; }
        // Apply sensor flip BEFORE S_FMT so the Bayer pattern is negotiated
        // (and the ISP latches it) in its flipped RGGB->BGGR phase.
        if (want_hflip_ || want_vflip_)
            sensor_flip_ok_ = trySetFlip(want_hflip_, want_vflip_);
        if (!setFormat())      { fprintf(stderr,"[V4L2] setFormat failed\n");   return false; }
        if (!requestBuffers()) { fprintf(stderr,"[V4L2] reqBufs failed\n");     return false; }
        if (!queueAllBuffers()){ fprintf(stderr,"[V4L2] queueBufs failed\n");   return false; }
        fprintf(stdout, "[V4L2] Device ready: %dx%d planes=%u\n",
                width_, height_, nplanes_);
        return true;
    }

    bool start() {
        if (streaming_) return true;
        v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        if (ioctl(fd_, VIDIOC_STREAMON, &type) == -1) {
            perror("[V4L2] STREAMON"); return false;
        }
        streaming_ = true;
        return true;
    }

    bool stop() {
        if (!streaming_) return true;
        v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        if (ioctl(fd_, VIDIOC_STREAMOFF, &type) == -1) {
            perror("[V4L2] STREAMOFF"); return false;
        }
        streaming_ = false;
        return true;
    }

    bool waitFrame(const unsigned char** plane_data,
                   std::vector<size_t>& plane_sizes,
                   unsigned& buf_idx) {
        if (!streaming_) return false;
        fd_set fds; FD_ZERO(&fds); FD_SET(fd_, &fds);
        timeval tv = {0, 500000};
        if (select(fd_ + 1, &fds, nullptr, nullptr, &tv) <= 0) return false;

        v4l2_buffer buf; v4l2_plane planes[3];
        memset(&buf, 0, sizeof(buf));
        buf.type     = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        buf.memory   = V4L2_MEMORY_MMAP;
        buf.length   = nplanes_;
        buf.m.planes = planes;
        if (ioctl(fd_, VIDIOC_DQBUF, &buf) == -1) return false;

        buf_idx = buf.index;
        plane_sizes.resize(nplanes_);
        for (unsigned i = 0; i < nplanes_; i++) {
            plane_data[i]  = (const unsigned char*)buffers_[buf.index].start[i];
            plane_sizes[i] = planes[i].bytesused;
        }
        return true;
    }

    bool releaseFrame(unsigned buf_idx) {
        v4l2_buffer buf; v4l2_plane planes[3];
        memset(&buf, 0, sizeof(buf));
        buf.type     = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        buf.memory   = V4L2_MEMORY_MMAP;
        buf.index    = buf_idx;
        buf.length   = nplanes_;
        buf.m.planes = planes;
        return ioctl(fd_, VIDIOC_QBUF, &buf) != -1;
    }

    int      width()    const { return width_;    }
    int      height()   const { return height_;   }
    unsigned nplanes()  const { return nplanes_;  }
    int      video_id() const { return video_id_; }

#ifdef USE_AWI_SP
    void set_fps_isp(AWIspApi* isp, int isp_id) {
        isp->ispSetFpsRanage(isp_id, fps_);
        fprintf(stdout, "[ISP] FPS set to %d\n", fps_);
    }
#endif

    bool trySetFlip(bool hflip, bool vflip) {
        if (fd_ < 0) return false;
        v4l2_control ctrl;
        memset(&ctrl, 0, sizeof(ctrl));

        ctrl.id = V4L2_CID_HFLIP;
        ctrl.value = hflip ? 1 : 0;
        bool hflip_ok = (ioctl(fd_, VIDIOC_S_CTRL, &ctrl) == 0);

        ctrl.id = V4L2_CID_VFLIP;
        ctrl.value = vflip ? 1 : 0;
        bool vflip_ok = (ioctl(fd_, VIDIOC_S_CTRL, &ctrl) == 0);

        if (hflip_ok && vflip_ok) {
            fprintf(stdout, "[V4L2] Sensor flip (hflip+vflip) set OK — "
                    "software flip disabled\n");
            return true;
        }
        fprintf(stdout, "[V4L2] Sensor flip NOT supported (hflip=%s vflip=%s), "
                "falling back to cv::flip\n",
                hflip_ok ? "OK" : "FAIL", vflip_ok ? "OK" : "FAIL");
        return false;
    }

private:
    struct Buffer { void* start[3]; size_t length[3]; };

    std::string device_;
    int width_, height_, fd_, fps_, video_id_;
    bool streaming_;
    bool want_hflip_ = false, want_vflip_ = false, sensor_flip_ok_ = false;
    unsigned nplanes_;
    Buffer buffers_[BUF_COUNT];

    static int parseVideoId(const std::string& dev) {
        size_t pos = dev.rfind("video");
        if (pos != std::string::npos) return std::stoi(dev.substr(pos + 5));
        return -1;
    }

    bool openDevice() {
        fd_ = open(device_.c_str(), O_RDWR | O_NONBLOCK, 0);
        if (fd_ < 0) { perror("[V4L2] open"); return false; }

        // Required for sunxi-vin
        v4l2_input inp; memset(&inp, 0, sizeof(inp));
        inp.index = 0;
        ioctl(fd_, VIDIOC_S_INPUT, &inp);

        v4l2_streamparm parm; memset(&parm, 0, sizeof(parm));
        parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        parm.parm.capture.timeperframe.numerator   = 1;
        parm.parm.capture.timeperframe.denominator = (unsigned)fps_;
        parm.parm.capture.capturemode = 0x0002; // V4L2_MODE_VIDEO
        ioctl(fd_, VIDIOC_S_PARM, &parm);
        ioctl(fd_, VIDIOC_G_PARM, &parm);
        if (parm.parm.capture.timeperframe.denominator > 0 &&
            parm.parm.capture.timeperframe.numerator > 0) {
            int actual = parm.parm.capture.timeperframe.denominator /
                         parm.parm.capture.timeperframe.numerator;
            if (actual != fps_)
                fprintf(stdout, "[V4L2] Requested %d fps, driver reports %d fps "
                        "(sensor is mode-locked)\n", fps_, actual);
        }
        return true;
    }

    bool setFormat() {
        v4l2_format fmt; memset(&fmt, 0, sizeof(fmt));
        fmt.type                   = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        fmt.fmt.pix_mp.width       = (unsigned)width_;
        fmt.fmt.pix_mp.height      = (unsigned)height_;
        fmt.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_BGR24;
        fmt.fmt.pix_mp.field       = V4L2_FIELD_NONE;
        if (ioctl(fd_, VIDIOC_S_FMT, &fmt) == -1) { perror("[V4L2] S_FMT"); return false; }
        ioctl(fd_, VIDIOC_G_FMT, &fmt);
        nplanes_ = fmt.fmt.pix_mp.num_planes;
        width_   = (int)fmt.fmt.pix_mp.width;
        height_  = (int)fmt.fmt.pix_mp.height;
        fprintf(stdout, "[V4L2] Format negotiated: %dx%d planes=%u pixfmt=0x%x\n",
                width_, height_, nplanes_, fmt.fmt.pix_mp.pixelformat);
        return true;
    }

    bool requestBuffers() {
        v4l2_requestbuffers req; memset(&req, 0, sizeof(req));
        req.count  = BUF_COUNT;
        req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        req.memory = V4L2_MEMORY_MMAP;
        if (ioctl(fd_, VIDIOC_REQBUFS, &req) == -1) { perror("[V4L2] REQBUFS"); return false; }

        for (unsigned i = 0; i < BUF_COUNT; i++) {
            v4l2_buffer buf; v4l2_plane planes[3];
            memset(&buf, 0, sizeof(buf));
            buf.type     = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
            buf.memory   = V4L2_MEMORY_MMAP;
            buf.index    = i;
            buf.length   = nplanes_;
            buf.m.planes = planes;
            if (ioctl(fd_, VIDIOC_QUERYBUF, &buf) == -1) { perror("[V4L2] QUERYBUF"); return false; }
            for (unsigned j = 0; j < nplanes_; j++) {
                buffers_[i].length[j] = planes[j].length;
                buffers_[i].start[j]  = mmap(nullptr, planes[j].length,
                    PROT_READ | PROT_WRITE, MAP_SHARED, fd_, planes[j].m.mem_offset);
                if (buffers_[i].start[j] == MAP_FAILED) { perror("[V4L2] mmap"); return false; }
            }
        }
        return true;
    }

    bool queueAllBuffers() {
        for (unsigned i = 0; i < BUF_COUNT; i++) {
            v4l2_buffer buf; v4l2_plane planes[3];
            memset(&buf, 0, sizeof(buf));
            buf.type     = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
            buf.memory   = V4L2_MEMORY_MMAP;
            buf.index    = i;
            buf.length   = nplanes_;
            buf.m.planes = planes;
            if (ioctl(fd_, VIDIOC_QBUF, &buf) == -1) { perror("[V4L2] QBUF"); return false; }
        }
        return true;
    }

    bool releaseBuffers() {
        v4l2_requestbuffers req; memset(&req, 0, sizeof(req));
        req.count  = 0;
        req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        req.memory = V4L2_MEMORY_MMAP;
        ioctl(fd_, VIDIOC_REQBUFS, &req);
        for (unsigned i = 0; i < BUF_COUNT; i++)
            for (unsigned j = 0; j < nplanes_; j++)
                if (buffers_[i].start[j] != MAP_FAILED && buffers_[i].start[j]) {
                    munmap(buffers_[i].start[j], buffers_[i].length[j]);
                    buffers_[i].start[j] = MAP_FAILED;
                }
        return true;
    }
};

// ── ROS 2 Node ────────────────────────────────────────────────────────────────

class Imx219CameraNode : public rclcpp::Node {
public:
    Imx219CameraNode() : Node("imx219_camera_node") {
        declare_parameter("topic",            "/camera_1/image_raw");
        declare_parameter("source_type",      "v4l2");
        declare_parameter("sensor",           "imx219");
        declare_parameter("fps",              30);
        declare_parameter("width",            1280);
        declare_parameter("height",           960);
        declare_parameter("resize_w",         0);
        declare_parameter("resize_h",         0);
        declare_parameter("video_device",     "/dev/video8");
        declare_parameter("format",           "BGR24");
        declare_parameter("io_mode",          "mmap");
        declare_parameter("qos_reliable",     true);
        declare_parameter("calibration_file", "");
        declare_parameter("enable_isp",       true);
        declare_parameter("frame_id",         "camera_optical_1");
        declare_parameter("flip_180",         false);
        declare_parameter("device",           "");
        declare_parameter("in_size",          "");

        topic_        = get_parameter("topic").as_string();
        source_type_  = get_parameter("source_type").as_string();
        sensor_       = get_parameter("sensor").as_string();
        fps_          = get_parameter("fps").as_int();
        cap_w_        = get_parameter("width").as_int();
        cap_h_        = get_parameter("height").as_int();
        resize_w_     = get_parameter("resize_w").as_int();
        resize_h_     = get_parameter("resize_h").as_int();
        video_device_ = get_parameter("video_device").as_string();
        format_       = get_parameter("format").as_string();
        io_mode_      = get_parameter("io_mode").as_string();
        enable_isp_   = get_parameter("enable_isp").as_bool();
        frame_id_     = get_parameter("frame_id").as_string();
        flip_180_     = get_parameter("flip_180").as_bool();

        if (source_type_ != "v4l2") {
            RCLCPP_WARN(get_logger(),
                "A733 backend supports only source_type='v4l2'; requested '%s', using v4l2",
                source_type_.c_str());
            source_type_ = "v4l2";
        }

        if (format_ == "bgr24") {
            format_ = "BGR24";
        } else if (format_ != "BGR24") {
            RCLCPP_WARN(get_logger(),
                "A733 backend currently captures BGR24; requested format '%s' will be ignored",
                format_.c_str());
            format_ = "BGR24";
        }

        if (io_mode_ != "mmap" && io_mode_ != "auto" && !io_mode_.empty()) {
            RCLCPP_WARN(get_logger(),
                "A733 backend currently uses mmap buffers; requested io_mode '%s' will be ignored",
                io_mode_.c_str());
            io_mode_ = "mmap";
        }
        if (io_mode_.empty() || io_mode_ == "auto")
            io_mode_ = "mmap";

        std::string legacy_device = get_parameter("device").as_string();
        if (!legacy_device.empty() && video_device_ == "/dev/video8")
            video_device_ = legacy_device;

        std::string legacy_in_size = get_parameter("in_size").as_string();
        if (!legacy_in_size.empty() && cap_w_ == 1280 && cap_h_ == 960) {
            if (!parse_size(legacy_in_size, cap_w_, cap_h_)) {
                RCLCPP_WARN(get_logger(),
                    "Invalid in_size '%s', using width/height defaults 1280x960",
                    legacy_in_size.c_str());
                cap_w_ = 1280; cap_h_ = 960;
            }
        }

        if (cap_w_ <= 0 || cap_h_ <= 0) {
            RCLCPP_WARN(get_logger(), "Invalid width/height %dx%d, using 1280x960",
                cap_w_, cap_h_);
            cap_w_ = 1280; cap_h_ = 960;
        }

        out_w_ = (resize_w_ > 0) ? resize_w_ : cap_w_;
        out_h_ = (resize_h_ > 0) ? resize_h_ : cap_h_;

        bool reliable = get_parameter("qos_reliable").as_bool();
        auto qos = rclcpp::QoS(1);
        reliable ? qos.reliable() : qos.best_effort();

        // Derive camera_info topic from image topic namespace.
        // /camera_1/image_raw  →  /camera_1/camera_info  (ROS convention)
        // Falls back to <topic>_info if topic has no namespace.
        std::string info_topic;
        auto slash = topic_.rfind('/');
        if (slash != std::string::npos && slash > 0)
            info_topic = topic_.substr(0, slash) + "/camera_info";
        else
            info_topic = topic_ + "_info";

        img_pub_  = create_publisher<sensor_msgs::msg::Image>(topic_, qos);

        // camera_info: RELIABLE + transient_local (latched). The aruco
        // subscribers (aruco_detect/aruco_loc) request transient_local, and a
        // volatile publisher is QoS-incompatible with them — the intrinsics
        // would never arrive at all.
        auto info_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
        info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(info_topic, info_qos);

        std::string cal_file = get_parameter("calibration_file").as_string();
        cinfo_mgr_ = std::make_shared<camera_info_manager::CameraInfoManager>(
            this, sensor_,
            cal_file.empty() ? "" : "file://" + cal_file);

        RCLCPP_INFO(get_logger(),
            "\n=================================================="
            "\n  imx219_camera_node"
            "\n=================================================="
            "\n  source_type: %s"
            "\n  sensor:      %s"
            "\n  video_device:%s"
            "\n  capture:     %dx%d @ %d fps"
            "\n  output:      %dx%d"
            "\n  format:      %s"
            "\n  io_mode:     %s"
            "\n  topic:       %s"
            "\n  topic_info:  %s"
            "\n  QoS:         %s"
            "\n  ISP 3A:      %s"
            "\n  calibration: %s"
            "\n  flip_180:    %s"
            "\n==================================================",
            source_type_.c_str(),
            sensor_.c_str(),
            video_device_.c_str(),
            cap_w_, cap_h_, fps_,
            out_w_, out_h_,
            format_.c_str(),
            io_mode_.c_str(),
            topic_.c_str(), info_topic.c_str(),
            reliable ? "RELIABLE" : "BEST_EFFORT",
            enable_isp_ ? "enabled (AWIspApi)" : "disabled",
            cal_file.empty() ? "none" : cal_file.c_str(),
            flip_180_ ? "enabled" : "disabled");

        running_ = true;
        capture_thread_ = std::thread(&Imx219CameraNode::capture_loop, this);
    }

    ~Imx219CameraNode() {
        stop();
    }

    // Public stop — called from main on shutdown
    void stop() {
        running_ = false;
        // capture_thread is in select() with 500ms timeout, will exit cleanly
        if (capture_thread_.joinable()) {
            // Wait max 2 seconds, then detach to avoid blocking shutdown
            auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
            while (capture_thread_.joinable()) {
                if (std::chrono::steady_clock::now() > deadline) {
                    capture_thread_.detach();
                    break;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
            if (capture_thread_.joinable())
                capture_thread_.join();
        }
    }

private:
    void capture_loop() {
        try_set_realtime(15);

        while (running_) {
            // ── Init camera (exactly as in camera_shm_host) ──────────────────
            V4L2Camera cam(video_device_, cap_w_, cap_h_, fps_);
            // Request sensor flip BEFORE init() so it is applied before S_FMT
            // (needed for correct Bayer phase / colors — see requestFlip()).
            if (flip_180_)
                cam.requestFlip(true, true);
            if (!cam.init()) {
                RCLCPP_ERROR(get_logger(), "Camera init failed, retrying in 2s...");
                std::this_thread::sleep_for(std::chrono::seconds(2));
                continue;
            }

            // ── STREAMON (must come before ISP init — sunxi-vin requirement) ─
            if (!cam.start()) {
                RCLCPP_ERROR(get_logger(), "STREAMON failed, retrying...");
                std::this_thread::sleep_for(std::chrono::seconds(2));
                continue;
            }

            // ── Sensor-level flip result (applied inside init(), pre-S_FMT) ──
            bool sensor_flip_ok = cam.sensorFlipOk();

            // ── ISP init after STREAMON (verbatim from camera_shm_host) ──────
            bool isp_ok = false;
#ifdef USE_AWI_SP
            AWIspApi* isp    = nullptr;
            int       isp_id = -1;

            if (enable_isp_) {
                isp = CreateAWIspApi();
                if (isp) {
                    if (isp->ispApiInit() >= 0) {
                        isp_id = isp->ispGetIspId(cam.video_id());
                        if (isp_id >= 0) {
                            fprintf(stdout, "[ISP] Initialized: video_id=%d isp_id=%d\n",
                                    cam.video_id(), isp_id);
                            int ret = isp->ispStart(isp_id);
                            if (ret == 0) {
                                isp->ispSetFpsRanage(isp_id, fps_);
                                fprintf(stdout, "[ISP] Started isp_id=%d\n", isp_id);
                                isp_ok = true;
                            } else {
                                fprintf(stderr, "[ISP] ispStart failed: %d\n", ret);
                            }
                        } else {
                            fprintf(stderr, "[ISP] ispGetIspId failed for video%d\n", cam.video_id());
                        }
                    } else {
                        fprintf(stderr, "[ISP] ispApiInit failed\n");
                    }
                    if (!isp_ok) {
                        isp->ispApiUnInit();
                        DestroyAWIspApi(isp);
                        isp = nullptr;
                    }
                } else {
                    fprintf(stderr, "[ISP] CreateAWIspApi failed\n");
                }
                if (!isp_ok)
                    RCLCPP_WARN(get_logger(), "ISP init failed — no 3A (dark image expected)");
            }
#endif

            RCLCPP_INFO(get_logger(), "Streaming started%s",
                        isp_ok ? " with ISP 3A" : " (no ISP)");

            uint64_t frame_count = 0;
            int64_t  t0 = now_ms();

            while (running_) {
                const unsigned char* plane_data[3] = {nullptr, nullptr, nullptr};
                std::vector<size_t>  plane_sizes;
                unsigned buf_idx = 0;

                if (!cam.waitFrame(plane_data, plane_sizes, buf_idx))
                    continue;

                // Build cv::Mat — single plane BGR3
                cv::Mat bgr(cam.height(), cam.width(), CV_8UC3,
                            const_cast<unsigned char*>(plane_data[0]));

                // Flip 180 deg if camera is mounted upside-down
                // (skip if sensor already flips — done via V4L2 ctrl above)
                if (flip_180_ && !sensor_flip_ok)
                    cv::rotate(bgr, bgr, cv::ROTATE_180);

                // Resize if requested
                cv::Mat out_img;
                if (out_w_ != cam.width() || out_h_ != cam.height())
                    cv::resize(bgr, out_img, cv::Size(out_w_, out_h_));
                else
                    out_img = bgr;

                auto stamp = this->now();

                // Publish Image
                auto img_msg = cv_bridge::CvImage(
                    std_msgs::msg::Header{}, "bgr8", out_img).toImageMsg();
                img_msg->header.stamp    = stamp;
                img_msg->header.frame_id = frame_id_;
                img_pub_->publish(*img_msg);

                // Publish CameraInfo
                auto info = cinfo_mgr_->getCameraInfo();
                info.header.stamp    = stamp;
                info.header.frame_id = frame_id_;
                info.width  = (uint32_t)out_w_;
                info.height = (uint32_t)out_h_;
                info_pub_->publish(info);

                cam.releaseFrame(buf_idx);
                ++frame_count;

                // Stats every 5s
                int64_t elapsed = now_ms() - t0;
                if (elapsed >= 5000) {
                    RCLCPP_INFO(get_logger(), "%.1f fps | frames=%lu | ISP=%s",
                        frame_count * 1000.0 / elapsed, frame_count,
                        isp_ok ? "on" : "off");
                    frame_count = 0;
                    t0 = now_ms();
                }
            }

            // ── Cleanup ISP (verbatim from camera_shm_host destructor) ───────
#ifdef USE_AWI_SP
            if (isp_ok && isp) {
                isp->ispWaitToExit(isp_id);
                isp->ispApiUnInit();
                DestroyAWIspApi(isp);
                isp = nullptr;
            }
#endif
            cam.stop();
        }
    }

    // Parameters
    std::string topic_, source_type_, sensor_, video_device_, format_, io_mode_, frame_id_;
    int fps_, cap_w_, cap_h_, out_w_, out_h_, resize_w_, resize_h_;
    bool enable_isp_, flip_180_;

    // ROS
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr      img_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
    std::shared_ptr<camera_info_manager::CameraInfoManager>    cinfo_mgr_;

    std::thread       capture_thread_;
    std::atomic<bool> running_{false};
};

// ── main ──────────────────────────────────────────────────────────────────────

static std::shared_ptr<Imx219CameraNode> g_node;

static void sig_handler(int) {
    if (g_node) g_node->stop();
    rclcpp::shutdown();
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    signal(SIGINT,  sig_handler);
    signal(SIGTERM, sig_handler);

    g_node = std::make_shared<Imx219CameraNode>();
    rclcpp::spin(g_node);

    // Ensure thread is stopped even if spin exits normally
    if (g_node) g_node->stop();

    return 0;
}
