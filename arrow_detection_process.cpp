#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <vector>
#include <cmath>
#include <string>
#include <numeric>
#include <algorithm>
#include <unistd.h>
#include <sys/wait.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <signal.h>
#include <chrono>
#include <thread>
#include <algorithm>
#include <iomanip>
#include <sstream>
#include <ctime>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <mutex>
#include <atomic>

#include "shared_memory.h"

class HTTPStreamServer {
private:
    int server_socket;
    std::atomic<bool> is_running;
    std::thread server_thread;
    std::mutex frame_mutex;
    cv::Mat current_frame;
    int port;
    
public:
    HTTPStreamServer(int port = 8080) : server_socket(-1), is_running(false), port(port) {}
    
    bool start() {
        server_socket = socket(AF_INET, SOCK_STREAM, 0);
        if (server_socket < 0) {
            std::cerr << "Failed to create socket for HTTP streaming" << std::endl;
            return false;
        }
        
        // Allow socket reuse
        int opt = 1;
        setsockopt(server_socket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
        
        struct sockaddr_in server_addr;
        server_addr.sin_family = AF_INET;
        server_addr.sin_addr.s_addr = INADDR_ANY;
        server_addr.sin_port = htons(port);
        
        if (bind(server_socket, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
            std::cerr << "Failed to bind socket on port " << port << std::endl;
            close(server_socket);
            return false;
        }
        
        if (listen(server_socket, 5) < 0) {
            std::cerr << "Failed to listen on socket" << std::endl;
            close(server_socket);
            return false;
        }
        
        is_running = true;
        server_thread = std::thread(&HTTPStreamServer::serverLoop, this);
                
        return true;
    }
    
    void stop() {
        is_running = false;
        if (server_socket >= 0) {
            close(server_socket);
            server_socket = -1;
        }
        if (server_thread.joinable()) {
            server_thread.join();
        }
    }
    
    void updateFrame(const cv::Mat& frame) {
        std::lock_guard<std::mutex> lock(frame_mutex);
        current_frame = frame.clone();
    }
    
private:
    void serverLoop() {
        while (is_running) {
            struct sockaddr_in client_addr;
            socklen_t client_len = sizeof(client_addr);
            
            int client_socket = accept(server_socket, (struct sockaddr*)&client_addr, &client_len);
            if (client_socket < 0) {
                if (is_running) {
                    std::cerr << "Failed to accept client connection" << std::endl;
                }
                continue;
            }
            
            // Handle client in a separate thread
            std::thread client_thread(&HTTPStreamServer::handleClient, this, client_socket);
            client_thread.detach();
        }
    }
    
    void handleClient(int client_socket) {
        char buffer[1024];
        recv(client_socket, buffer, sizeof(buffer), 0);
        
        // Parse the HTTP request
        std::string request(buffer);
        
        if (request.find("GET /stream") != std::string::npos) {
            // Send MJPEG stream
            sendMJPEGStream(client_socket);
        } else if (request.find("GET /") != std::string::npos) {
            // Send simple HTML page
            sendHTMLPage(client_socket);
        }
        
        close(client_socket);
    }
    
    void sendHTMLPage(int client_socket) {
        std::string html = 
            "HTTP/1.1 200 OK\r\n"
            "Content-Type: text/html\r\n"
            "Connection: close\r\n\r\n"
            "<!DOCTYPE html>\n"
            "<html>\n"
            "<head>\n"
            "    <title>Wall-E Arrow Detection Live Stream</title>\n"
            "    <style>\n"
            "        body { background: #000; color: #fff; font-family: Arial; text-align: center; }\n"
            "        h1 { color: #0f0; }\n"
            "        img { border: 2px solid #0f0; margin: 20px; }\n"
            "        .info { margin: 20px; padding: 10px; background: #333; border-radius: 5px; }\n"
            "    </style>\n"
            "</head>\n"
            "<body>\n"
            "    <h1>🤖 Wall-E Arrow Detection Live Stream</h1>\n"
            "    <div class=\"info\">\n"
            "        <p>Live camera feed with arrow detection overlay</p>\n"
            "        <p>Green boxes indicate detected arrows with direction and confidence</p>\n"
            "    </div>\n"
            "    <img src=\"/stream\" width=\"640\" height=\"480\" alt=\"Live Stream\">\n"
            "    <div class=\"info\">\n"
            "        <p>Stream Resolution: 640x480 | Refresh automatically</p>\n"
            "    </div>\n"
            "</body>\n"
            "</html>";
        
        send(client_socket, html.c_str(), html.length(), 0);
    }
    
    void sendMJPEGStream(int client_socket) {
        // Send MJPEG headers
        std::string headers = 
            "HTTP/1.1 200 OK\r\n"
            "Content-Type: multipart/x-mixed-replace; boundary=--boundary\r\n"
            "Cache-Control: no-cache\r\n"
            "Pragma: no-cache\r\n"
            "Connection: close\r\n\r\n";
        
        send(client_socket, headers.c_str(), headers.length(), 0);
        
        while (is_running) {
            cv::Mat frame_to_send;
            {
                std::lock_guard<std::mutex> lock(frame_mutex);
                if (current_frame.empty()) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                    continue;
                }
                frame_to_send = current_frame.clone();
            }
            
            // Encode frame as JPEG
            std::vector<uchar> buffer;
            std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 85};
            
            if (!cv::imencode(".jpg", frame_to_send, buffer, params)) {
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
                continue;
            }
            
            // Send frame boundary and headers
            std::string frame_header = 
                "\r\n--boundary\r\n"
                "Content-Type: image/jpeg\r\n"
                "Content-Length: " + std::to_string(buffer.size()) + "\r\n\r\n";
            
            if (send(client_socket, frame_header.c_str(), frame_header.length(), MSG_NOSIGNAL) < 0) {
                break;
            }
            
            // Send frame data
            if (send(client_socket, buffer.data(), buffer.size(), MSG_NOSIGNAL) < 0) {
                break;
            }
            
            // Limit to ~20 FPS for streaming
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }
};

class ArrowDetectionProcess {
private:
    bool running;
    SharedMemoryManager shm;
    BalanceData* balance_data;
    
    std::string lastDetectedDirection;
    std::chrono::steady_clock::time_point lastDetectionTime;
    cv::VideoCapture cap;
    
    // HTTP streaming
    std::unique_ptr<HTTPStreamServer> stream_server;
    bool streaming_enabled;
    int streaming_port;
    bool standalone_mode;
    
    // Standalone mode tracking variables
    ArrowDirection standalone_detected_direction;
    float standalone_confidence;
    bool standalone_detection_active;
    
    // HSV ranges for red color detection (tightened for better precision)
    cv::Scalar lower_red1 = cv::Scalar(0, 140, 80);
    cv::Scalar upper_red1 = cv::Scalar(8, 255, 255);
    cv::Scalar lower_red2 = cv::Scalar(172, 140, 80);
    cv::Scalar upper_red2 = cv::Scalar(180, 255, 255);
    
    // Arrow detection parameters
    struct ArrowFeatures {
        double triangularity_score;    // How triangular is the arrow tip
        double symmetry_score;         // How symmetric is the arrow
        double shaft_score;           // How well-defined is the arrow shaft
        double edge_density;          // Density of edges in expected arrow regions
        double corner_count;          // Number of significant corners (should be 3-7 for arrows)
        cv::Point2f tip_point;        // Detected tip location
        cv::Point2f base_center;      // Center of the arrow base
        double tip_confidence;        // Confidence that tip_point is actually the tip
    };
    
public:
    ArrowDetectionProcess(bool enable_streaming = false, int port = 8080) : 
        running(false), shm(false), balance_data(nullptr), 
        lastDetectedDirection("NONE"), streaming_enabled(enable_streaming), 
        streaming_port(port), standalone_mode(false),
        standalone_detected_direction(ARROW_NONE), standalone_confidence(0.0f),
        standalone_detection_active(false) {
        
        if (streaming_enabled) {
            stream_server = std::make_unique<HTTPStreamServer>(streaming_port);
        }
    }
    
    bool initializeSharedMemory() {
        // If streaming is enabled, try standalone mode first
        if (streaming_enabled) {
            std::cout << "Attempting to connect to Wall-E shared memory system..." << std::endl;
            
            // Try to connect to shared memory (quick attempt)
            int retry_count = 0;
            while (!shm.initialize() && retry_count < 10) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                retry_count++;
            }
            
            if (retry_count >= 10) {
                std::cout << "Shared memory not available. Running in standalone streaming mode." << std::endl;
                standalone_mode = true;
                balance_data = nullptr;
                return true;
            } else {
                std::cout << "Connected to Wall-E shared memory system" << std::endl;
                balance_data = shm.getData();
                balance_data->arrow_detection_active = true;
                balance_data->arrow_detection_enabled = true;
                standalone_mode = false;
                return true;
            }
        } else {
            // Normal mode - require shared memory
            int retry_count = 0;
            while (!shm.initialize() && retry_count < 50) {
                std::cout << "Waiting for shared memory system..." << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                retry_count++;
            }
            
            if (retry_count >= 50) {
                std::cerr << "Failed to connect to shared memory. Are other processes running?" << std::endl;
                return false;
            }
            
            balance_data = shm.getData();
            balance_data->arrow_detection_active = true;
            balance_data->arrow_detection_enabled = true;
            standalone_mode = false;
            
            std::cout << "Connected to Wall-E shared memory system" << std::endl;
            return true;
        }
    }
    
    // Helper functions for standalone/shared memory modes
    void setDetectedDirection(ArrowDirection direction) {
        if (standalone_mode) {
            standalone_detected_direction = direction;
        } else if (balance_data) {
            balance_data->detected_arrow_direction = direction;
        }
    }
    
    void setConfidence(float confidence) {
        if (standalone_mode) {
            standalone_confidence = confidence;
        } else if (balance_data) {
            balance_data->arrow_confidence = confidence;
        }
    }
    
    void setTimestamp(uint64_t timestamp) {
        if (!standalone_mode && balance_data) {
            balance_data->arrow_timestamp = timestamp;
        }
    }
    
    void setDetectionActive(bool active) {
        if (standalone_mode) {
            standalone_detection_active = active;
        } else if (balance_data) {
            balance_data->arrow_detection_active = active;
        }
    }
    
    ArrowDirection getDetectedDirection() {
        if (standalone_mode) {
            return standalone_detected_direction;
        } else if (balance_data) {
            return balance_data->detected_arrow_direction;
        }
        return ARROW_NONE;
    }
    
    float getConfidence() {
        if (standalone_mode) {
            return standalone_confidence;
        } else if (balance_data) {
            return balance_data->arrow_confidence;
        }
        return 0.0f;
    }
    
    bool isDetectionEnabled() {
        if (standalone_mode) {
            return true; // Always enabled in standalone mode
        } else if (balance_data) {
            return balance_data->arrow_detection_enabled;
        }
        return false;
    }
    
    bool isEmergencyStop() {
        if (standalone_mode) {
            return false; // No emergency stop in standalone mode
        } else if (balance_data) {
            return balance_data->emergency_stop;
        }
        return false;
    }
    
    float getBalanceAngle() {
        if (standalone_mode) {
            return 0.0f; // No balance angle in standalone mode
        } else if (balance_data) {
            return balance_data->kalman_angle;
        }
        return 0.0f;
    }
    
    bool initRpicamVideo() {
        // Start rpicam-vid to stream to stdout which we'll pipe to opencv
        std::string command = "rpicam-vid --timeout 0 --width 640 --height 480 "
                             "--framerate 15 --codec mjpeg --output - 2>/dev/null";
        
        // Use popen to start the process and get a pipe to read from
        FILE* pipe = popen(command.c_str(), "r");
        if (!pipe) {
            std::cerr << "Failed to start rpicam-vid" << std::endl;
            return false;
        }
        
        // Try to open with OpenCV VideoCapture using the pipe
        // This is tricky - let's try a different approach using a fifo
        pclose(pipe);
   
        // Create a named pipe (FIFO)
        std::string fifoPath = "/tmp/rpicam_arrow_fifo";
        
        // Remove existing fifo if it exists
        unlink(fifoPath.c_str());
        
        // Create new fifo
        if (mkfifo(fifoPath.c_str(), 0666) != 0) {
            std::cerr << "Failed to create FIFO" << std::endl;
            return false;
        }
        
        // Start rpicam-vid in background to write to fifo
        std::string fifoCommand = "rpicam-vid --timeout 0 --width 640 --height 480 "
                                 "--framerate 20 --codec mjpeg --output " + fifoPath + " 2>/dev/null &";
        
        int result = system(fifoCommand.c_str());
        if (result != 0) {
            std::cerr << "Failed to start rpicam-vid background process" << std::endl;
            return false;
        }
        
        // Give rpicam-vid time to start
        std::this_thread::sleep_for(std::chrono::seconds(2));
        
        // Try to open the FIFO with OpenCV
        cap.open(fifoPath);
        if (!cap.isOpened()) {
            std::cerr << "Failed to open FIFO with OpenCV" << std::endl;
            return false;
        }
        
        // Test reading a frame
        cv::Mat testFrame;
        for (int i = 0; i < 10; i++) {
            if (cap.read(testFrame) && !testFrame.empty()) {
                std::cout << "Successfully opened rpicam-vid stream: " 
                         << testFrame.cols << "x" << testFrame.rows << std::endl;
                return true;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        std::cerr << "Failed to read frames from rpicam-vid stream" << std::endl;
        return false;
    }
    
    cv::Mat createRedMask(const cv::Mat& hsv) {
        cv::Mat mask1, mask2, mask;
        
        cv::inRange(hsv, lower_red1, upper_red1, mask1);
        cv::inRange(hsv, lower_red2, upper_red2, mask2);
        
        mask = mask1 | mask2;
        
        // Use adaptive kernel size - smaller for better preservation of large shapes
        int kernelSize = std::max(3, std::min(7, hsv.rows / 100));  // 3-7 pixels based on image size
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(kernelSize, kernelSize));
        
        // Add aggressive erosion to separate connected arrows before closing/opening
        cv::Mat separationKernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
        cv::morphologyEx(mask, mask, cv::MORPH_ERODE, separationKernel, cv::Point(-1,-1), 3);  // Stronger erosion to separate
        cv::morphologyEx(mask, mask, cv::MORPH_DILATE, separationKernel, cv::Point(-1,-1), 2); // Dilate back partially
        
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel, cv::Point(-1,-1), 1);  // Reduced iterations
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
        
        return mask;
    }
    
    // Enhanced arrow feature analysis
    ArrowFeatures analyzeArrowFeatures(const std::vector<cv::Point>& contour, const cv::Mat& originalImage) {
        ArrowFeatures features = {};
        
        if (contour.size() < 5) return features;
        
        // Get basic geometric properties
        cv::Rect bbox = cv::boundingRect(contour);
        cv::Moments moments = cv::moments(contour);
        if (moments.m00 == 0) return features;
        
        cv::Point2f centroid(moments.m10 / moments.m00, moments.m01 / moments.m00);
        
        // 1. Corner Detection for triangularity analysis
        std::vector<cv::Point> approx;
        double epsilon = 0.02 * cv::arcLength(contour, true);
        cv::approxPolyDP(contour, approx, epsilon, true);
        features.corner_count = approx.size();
        
        // 2. Find potential tip point using corner analysis
        std::vector<double> corner_angles;
        for (size_t i = 0; i < approx.size(); i++) {
            cv::Point p1 = approx[(i - 1 + approx.size()) % approx.size()];
            cv::Point p2 = approx[i];
            cv::Point p3 = approx[(i + 1) % approx.size()];
            
            cv::Vec2f v1(p1.x - p2.x, p1.y - p2.y);
            cv::Vec2f v2(p3.x - p2.x, p3.y - p2.y);
            
            double angle = acos(v1.dot(v2) / (cv::norm(v1) * cv::norm(v2) + 1e-6));
            corner_angles.push_back(angle * 180.0 / CV_PI);
        }
        
        // Find sharpest corner (likely arrow tip)
        if (!corner_angles.empty()) {
            auto min_it = std::min_element(corner_angles.begin(), corner_angles.end());
            size_t tip_idx = std::distance(corner_angles.begin(), min_it);
            if (tip_idx < approx.size()) {
                features.tip_point = cv::Point2f(approx[tip_idx]);
                features.tip_confidence = 180.0 - *min_it;  // Sharper corners have higher confidence
            }
        }
        
        // 3. Triangularity Score - how triangular is the overall shape
        if (features.corner_count >= 3 && features.corner_count <= 7) {
            features.triangularity_score = 100.0 - abs(5.0 - features.corner_count) * 15.0; // Optimal 5 corners for arrow
            if (!corner_angles.empty()) {
                double min_angle = *std::min_element(corner_angles.begin(), corner_angles.end());
                if (min_angle < 60.0) { // Sharp tip
                    features.triangularity_score += 20.0;
                }
            }
        }
        
        // 4. Symmetry Analysis
        cv::RotatedRect fitted_rect = cv::fitEllipse(contour);
        double orientation = fitted_rect.angle * CV_PI / 180.0;
        
        // Check if shape is symmetric along major axis
        cv::Mat symmetry_mask = cv::Mat::zeros(bbox.height, bbox.width, CV_8UC1);
        std::vector<cv::Point> translated_contour;
        for (const auto& pt : contour) {
            translated_contour.push_back(cv::Point(pt.x - bbox.x, pt.y - bbox.y));
        }
        cv::fillPoly(symmetry_mask, std::vector<std::vector<cv::Point>>{translated_contour}, cv::Scalar(255));
        
        // Simple symmetry check - compare left and right halves
        cv::Rect left_half(0, 0, bbox.width / 2, bbox.height);
        cv::Rect right_half(bbox.width / 2, 0, bbox.width / 2, bbox.height);
        
        cv::Mat left_roi = symmetry_mask(left_half);
        cv::Mat right_roi = symmetry_mask(right_half);
        cv::flip(right_roi, right_roi, 1); // Mirror right half
        
        cv::Mat diff;
        cv::absdiff(left_roi, right_roi, diff);
        features.symmetry_score = 100.0 - (cv::sum(diff)[0] / (bbox.width * bbox.height / 2.0 * 255.0)) * 100.0;
        
        // 5. Shaft Score - arrows should have a defined shaft/body
        double aspect_ratio = (double)bbox.width / bbox.height;
        if (aspect_ratio > 1.2 || aspect_ratio < 0.8) { // Not too square
            features.shaft_score = std::min(80.0, aspect_ratio > 1.0 ? (aspect_ratio - 1.0) * 40.0 : (1.0 / aspect_ratio - 1.0) * 40.0);
        }
        
        // 6. Edge Density Analysis - arrows have specific edge patterns
        cv::Mat roi = originalImage(bbox);
        cv::Mat edges;
        cv::Canny(roi, edges, 50, 150);
        features.edge_density = cv::sum(edges)[0] / (bbox.area() * 255.0) * 100.0;
        
        // 7. Calculate base center (opposite to tip)
        if (features.tip_confidence > 0) {
            // Find point furthest from tip
            double max_dist = 0;
            for (const auto& pt : contour) {
                double dist = cv::norm(cv::Point2f(pt) - features.tip_point);
                if (dist > max_dist) {
                    max_dist = dist;
                    features.base_center = cv::Point2f(pt);
                }
            }
        }
        
        return features;
    }
    
    // Enhanced arrow shape validation using multiple features
    bool isValidArrowShape(const std::vector<cv::Point>& contour, double area, const cv::Mat& originalImage) {
        if (contour.size() < 5) return false;
        
        // Basic area and geometric checks (kept from original)
        int frameArea = 640 * 480; 
        double minAreaRatio = 0.005; // Reduced minimum size to 0.5%
        double maxAreaRatio = 0.95; 
        
        double minArea = frameArea * minAreaRatio;
        double maxArea = frameArea * maxAreaRatio;
        
        if (area < minArea || area > maxArea) return false;
        
        cv::Rect boundingRect = cv::boundingRect(contour);
        double aspectRatio = (double)boundingRect.width / boundingRect.height;
        
        // Relaxed aspect ratio for various arrow orientations
        if (aspectRatio < 0.2 || aspectRatio > 5.0) return false;
        
        // Enhanced feature analysis
        ArrowFeatures features = analyzeArrowFeatures(contour, originalImage);
        
        // Multi-criteria arrow validation
        int score = 0;
        
        // Corner count should be reasonable for arrows (3-7 corners)
        if (features.corner_count >= 3 && features.corner_count <= 7) score += 20;
        
        // Triangularity score
        if (features.triangularity_score > 50.0) score += 25;
        
        // Tip confidence (sharp corners suggest arrow tips)
        if (features.tip_confidence > 40.0) score += 30;
        
        // Symmetry score
        if (features.symmetry_score > 30.0) score += 15;
        
        // Edge density (arrows have distinctive edge patterns)
        if (features.edge_density > 10.0 && features.edge_density < 60.0) score += 10;
        
        // Original solidity check (kept for compatibility)
        std::vector<cv::Point> hull;
        cv::convexHull(contour, hull);
        double hullArea = cv::contourArea(hull);
        double solidity = area / hullArea;
        if (solidity >= 0.5 && solidity <= 0.95) score += 10;
        
        // Require a minimum combined score to be considered an arrow
        return score >= 70; // Out of 130 possible points
    }
    
    std::vector<cv::Point> mergeNearbyContours(const std::vector<std::vector<cv::Point>>& contours, 
                                              const std::vector<double>& areas) {
        if (contours.empty()) return {};
        
        // Use frame-relative area limits for consistency
        int frameArea = 640 * 480;
        double minArea = frameArea * 0.0005; // 0.05% of frame
        double maxArea = frameArea * 0.95;   // 95% of frame
        
        size_t largestIdx = 0;
        double largestArea = 0;
        
        for (size_t i = 0; i < contours.size(); i++) {
            if (areas[i] >= minArea && areas[i] <= maxArea && areas[i] > largestArea) {
                largestArea = areas[i];
                largestIdx = i;
            }
        }
        
        if (largestArea == 0) return {};
        
        std::vector<cv::Point> mergedContour = contours[largestIdx];
        cv::Rect mainRect = cv::boundingRect(mergedContour);
        
        for (size_t i = 0; i < contours.size(); i++) {
            if (i == largestIdx || areas[i] < 200) continue;
            
            cv::Rect currentRect = cv::boundingRect(contours[i]);
            
            int expandedWidth = mainRect.width * 1.5;
            int expandedHeight = mainRect.height * 1.5;
            cv::Rect expandedMain(
                mainRect.x - expandedWidth * 0.25,
                mainRect.y - expandedHeight * 0.25,
                expandedWidth,
                expandedHeight
            );
            
            cv::Rect intersection = expandedMain & currentRect;
            if (intersection.area() > 0) {
                mergedContour.insert(mergedContour.end(), contours[i].begin(), contours[i].end());
            }
        }
        
        return mergedContour;
    }
    
    // Enhanced confidence calculation using arrow features
    double calculateArrowConfidence(const std::vector<cv::Point>& contour, double area, const std::string& direction, const cv::Mat& originalImage) {
        if (contour.size() < 5 || direction == "UNKNOWN") return 0.0;
        
        cv::Rect boundingRect = cv::boundingRect(contour);
        double aspectRatio = (double)boundingRect.width / boundingRect.height;
        
        // Get enhanced arrow features
        ArrowFeatures features = analyzeArrowFeatures(contour, originalImage);
        
        double confidence = 0.0;
        
        // Factor 1: Arrow-specific geometric features (40% of total)
        confidence += features.triangularity_score * 0.2;  // 20% weight
        confidence += features.tip_confidence * 0.2;       // 20% weight
        
        // Factor 2: Shape consistency (25% of total)
        if (features.corner_count >= 3 && features.corner_count <= 7) {
            confidence += 15.0; // Good corner count
            if (features.corner_count == 5 || features.corner_count == 6) {
                confidence += 10.0; // Optimal corner count for arrows
            }
        }
        
        // Factor 3: Symmetry (15% of total)
        confidence += features.symmetry_score * 0.15;
        
        // Factor 4: Size appropriateness (20% of total) - heavily favor larger arrows
        int frameArea = 640 * 480;
        double areaRatio = area / frameArea;
        
        if (areaRatio >= 0.08) {
            confidence += 20.0; // Very large arrows
        } else if (areaRatio >= 0.04) {
            confidence += 18.0; // Large arrows
        } else if (areaRatio >= 0.02) {
            confidence += 15.0; // Medium-large arrows
        } else if (areaRatio >= 0.01) {
            confidence += 10.0; // Medium arrows
        } else if (areaRatio >= 0.005) {
            confidence += 5.0;  // Small arrows
        }
        
        // Factor 5: Aspect ratio appropriateness (10% of total)
        if (aspectRatio >= 1.2 && aspectRatio <= 3.0) {
            confidence += 10.0; // Ideal arrow proportions
        } else if (aspectRatio >= 0.5 && aspectRatio <= 4.0) {
            confidence += 7.0;  // Acceptable proportions
        } else {
            confidence += 3.0;  // Poor proportions
        }
        
        // Factor 6: Edge density bonus (5% of total)
        if (features.edge_density > 10.0 && features.edge_density < 50.0) {
            confidence += 5.0; // Good edge patterns
        }
        
        // Direction consistency bonus
        if (direction != "UNKNOWN" && features.tip_confidence > 30.0) {
            confidence += 5.0; // Clear directional indication
        }
        
        // Penalty for very complex shapes (likely not arrows)
        if (features.corner_count > 10) {
            confidence -= 15.0;
        }
        
        // Ensure confidence is within bounds
        return std::max(0.0, std::min(confidence, 100.0));
    }
    
    // Enhanced direction detection using arrow features
    std::string determineArrowDirection(const std::vector<cv::Point>& contour, const cv::Mat& originalImage) {
        if (contour.size() < 6) return "UNKNOWN";
        
        // Get arrow features including tip detection
        ArrowFeatures features = analyzeArrowFeatures(contour, originalImage);
        
        if (features.tip_confidence < 20.0) {
            return "UNKNOWN"; // Not confident enough about tip location
        }
        
        cv::Rect bbox = cv::boundingRect(contour);
        cv::Moments moments = cv::moments(contour);
        if (moments.m00 == 0) return "UNKNOWN";
        cv::Point2f centroid(moments.m10 / moments.m00, moments.m01 / moments.m00);
        
        // Use detected tip point to determine direction
        cv::Point2f tip = features.tip_point;
        cv::Point2f base = features.base_center;
        
        // Calculate direction vector from base to tip
        cv::Vec2f direction_vector = tip - base;
        double angle = atan2(direction_vector[1], direction_vector[0]) * 180.0 / CV_PI;
        
        // Normalize angle to 0-360 range
        if (angle < 0) angle += 360.0;
        
        // Determine direction based on angle ranges
        if (angle >= 315.0 || angle < 45.0) {
            return "RIGHT";
        } else if (angle >= 45.0 && angle < 135.0) {
            return "DOWN";
        } else if (angle >= 135.0 && angle < 225.0) {
            return "LEFT";
        } else if (angle >= 225.0 && angle < 315.0) {
            return "UP";
        }
        
        // Fallback to original method if tip-based detection fails
        bool isHorizontal = bbox.width > bbox.height * 1.2;
        bool isVertical = bbox.height > bbox.width * 1.2;
        
        // Find convex hull for clean extreme points
        std::vector<cv::Point> hull;
        cv::convexHull(contour, hull);
        
        cv::Point leftMost = hull[0], rightMost = hull[0];
        cv::Point topMost = hull[0], bottomMost = hull[0];
        
        for (const cv::Point& pt : hull) {
            if (pt.x < leftMost.x) leftMost = pt;
            if (pt.x > rightMost.x) rightMost = pt;
            if (pt.y < topMost.y) topMost = pt;
            if (pt.y > bottomMost.y) bottomMost = pt;
        }
        
        // Calculate distances from centroid to each extreme
        auto distance = [&](cv::Point pt) -> double {
            return sqrt(pow(pt.x - centroid.x, 2) + pow(pt.y - centroid.y, 2));
        };
        
        if (isHorizontal) {
            double leftDist = distance(leftMost);
            double rightDist = distance(rightMost);
            
            if (leftDist > rightDist * 1.15) {  // Increased threshold for better accuracy
                return "RIGHT";
            } else if (rightDist > leftDist * 1.15) {
                return "LEFT";
            }
            
        } else if (isVertical) {
            double topDist = distance(topMost);
            double bottomDist = distance(bottomMost);
            
            if (topDist > bottomDist * 1.15) {
                return "DOWN";
            } else if (bottomDist > topDist * 1.15) {
                return "UP";
            }
        }
        
        return "UNKNOWN";
    }
    
    std::vector<std::vector<cv::Point>> filterSpatiallyCloseContours(const std::vector<std::vector<cv::Point>>& contours) {
        if (contours.size() <= 1) return contours;
        
        // Calculate areas and centroids for all contours
        std::vector<double> areas;
        std::vector<cv::Point2f> centroids;
        
        for (const auto& contour : contours) {
            double area = cv::contourArea(contour);
            areas.push_back(area);
            
            cv::Moments moments = cv::moments(contour);
            if (moments.m00 > 0) {
                centroids.push_back(cv::Point2f(moments.m10 / moments.m00, moments.m01 / moments.m00));
            } else {
                centroids.push_back(cv::Point2f(0, 0));
            }
        }
        
        // Create list of indices sorted by area (largest first)
        std::vector<size_t> indices(contours.size());
        std::iota(indices.begin(), indices.end(), 0);
        std::sort(indices.begin(), indices.end(), [&](size_t a, size_t b) {
            return areas[a] > areas[b];
        });
        
        std::vector<bool> keep(contours.size(), true);
        
        // For each contour (starting from largest)
        for (size_t i = 0; i < indices.size(); ++i) {
            if (!keep[indices[i]]) continue;
            
            size_t currentIdx = indices[i];
            cv::Rect currentRect = cv::boundingRect(contours[currentIdx]);
            
            // Check all smaller contours
            for (size_t j = i + 1; j < indices.size(); ++j) {
                size_t otherIdx = indices[j];
                if (!keep[otherIdx]) continue;
                
                // Calculate distance between centroids
                double distance = cv::norm(centroids[currentIdx] - centroids[otherIdx]);
                
                // Calculate minimum separation required (based on bounding rectangle sizes)
                double minSeparation = std::max(currentRect.width, currentRect.height) * 0.5;
                
                // If too close and the other contour is significantly smaller, remove it
                if (distance < minSeparation && areas[otherIdx] < areas[currentIdx] * 0.5) {
                    keep[otherIdx] = false;
                }
            }
        }
        
        // Return only the contours we want to keep
        std::vector<std::vector<cv::Point>> filtered;
        for (size_t i = 0; i < contours.size(); ++i) {
            if (keep[i]) {
                filtered.push_back(contours[i]);
            }
        }
        
        return filtered;
    }
    
    ArrowDirection stringToArrowDirection(const std::string& direction) {
        if (direction == "LEFT") return ARROW_LEFT;
        if (direction == "RIGHT") return ARROW_RIGHT;
        if (direction == "UP") return ARROW_UP;
        if (direction == "DOWN") return ARROW_DOWN;
        if (direction == "UNKNOWN") return ARROW_UNKNOWN;
        return ARROW_NONE;
    }
    
    std::string arrowDirectionToString(ArrowDirection dir) {
        switch (dir) {
            case ARROW_LEFT: return "LEFT";
            case ARROW_RIGHT: return "RIGHT";
            case ARROW_UP: return "UP";
            case ARROW_DOWN: return "DOWN";
            case ARROW_UNKNOWN: return "UNKNOWN";
            default: return "NONE";
        }
    }
    
    bool processFrame(cv::Mat& frame) {
        return processFrameWithOverlay(frame, nullptr);
    }
    
    bool processFrameWithOverlay(cv::Mat& frame, cv::Mat* overlayFrame) {
        cv::Mat hsv, mask;
        
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
        mask = createRedMask(hsv);
        
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        
        if (contours.empty()) {
            // No arrows detected - reset detection data
            setDetectedDirection(ARROW_NONE);
            setConfidence(0.0f);
            setTimestamp(getCurrentTimeMicros());
            return false;
        }
        
        // Filter out contours that are spatially too close to larger ones
        contours = filterSpatiallyCloseContours(contours);
        
        if (contours.empty()) {
            setDetectedDirection(ARROW_NONE);
            setConfidence(0.0f);
            setTimestamp(getCurrentTimeMicros());
            return false;
        }
        
        std::vector<double> areas;
        for (const auto& contour : contours) {
            areas.push_back(cv::contourArea(contour));
        }
        
        std::vector<bool> processed(contours.size(), false);
        std::vector<std::vector<cv::Point>> arrowCandidates;
        
        for (size_t i = 0; i < contours.size(); i++) {
            if (processed[i] || areas[i] < 300) continue;
            
            std::vector<std::vector<cv::Point>> groupContours;
            std::vector<double> groupAreas;
            
            groupContours.push_back(contours[i]);
            groupAreas.push_back(areas[i]);
            processed[i] = true;
            
            cv::Rect mainRect = cv::boundingRect(contours[i]);
            
            for (size_t j = 0; j < contours.size(); j++) {
                if (processed[j] || areas[j] < 200) continue;
                
                cv::Rect currentRect = cv::boundingRect(contours[j]);
                
                int expandedWidth = std::max(mainRect.width, currentRect.width) * 2;
                int expandedHeight = std::max(mainRect.height, currentRect.height) * 2;
                
                cv::Rect expandedMain(
                    mainRect.x - expandedWidth * 0.25,
                    mainRect.y - expandedHeight * 0.25,
                    expandedWidth,
                    expandedHeight
                );
                
                cv::Rect intersection = expandedMain & currentRect;
                if (intersection.area() > 0) {
                    groupContours.push_back(contours[j]);
                    groupAreas.push_back(areas[j]);
                    processed[j] = true;
                    mainRect = mainRect | currentRect;
                }
            }
            
            std::vector<cv::Point> mergedContour = mergeNearbyContours(groupContours, groupAreas);
            if (!mergedContour.empty()) {
                arrowCandidates.push_back(mergedContour);
            }
        }
        
        // Find the biggest valid arrow among all candidates
        size_t bestArrowIndex = SIZE_MAX;
        double bestArrowArea = 0.0;
        double bestConfidence = 0.0;
        std::string bestDirection = "UNKNOWN";
        
        // First pass: evaluate all arrow candidates and find the biggest valid one
        for (size_t i = 0; i < arrowCandidates.size(); i++) {
            double area = cv::contourArea(arrowCandidates[i]);
            
            if (!isValidArrowShape(arrowCandidates[i], area, frame)) continue;
            
            std::string direction = determineArrowDirection(arrowCandidates[i], frame);
            
            if (direction != "UNKNOWN") {
                // Calculate confidence for this detection
                double confidence = calculateArrowConfidence(arrowCandidates[i], area, direction, frame);
                
                // Only consider arrows with at least 75% confidence (increased threshold)
                if (confidence >= 75.0) {
                    // Prefer larger arrows
                    if (area > bestArrowArea) {
                        bestArrowIndex = i;
                        bestArrowArea = area;
                        bestConfidence = confidence;
                        bestDirection = direction;
                    }
                }
            }
        }
        
        // Update shared memory with detection results
        if (bestArrowIndex != SIZE_MAX) {
            auto currentTime = std::chrono::steady_clock::now();
            if (bestDirection != lastDetectedDirection || 
                std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - lastDetectionTime).count() > 50) {
                
                // Update detection data
                setDetectedDirection(stringToArrowDirection(bestDirection));
                setConfidence(bestConfidence);
                setTimestamp(getCurrentTimeMicros());
                
                // Get current timestamp for logging
                auto now = std::chrono::system_clock::now();
                auto time_t = std::chrono::system_clock::to_time_t(now);
                auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
                
                // Format timestamp as HH:MM:SS.mmm
                std::stringstream timestamp;
                timestamp << std::put_time(std::localtime(&time_t), "%H:%M:%S");
                timestamp << "." << std::setfill('0') << std::setw(3) << ms.count();
                
                std::cout << "\n=== ARROW DETECTED ==="
                         << "\nTimestamp: " << timestamp.str()
                         << "\nDirection: " << bestDirection
                         << "\nConfidence: " << std::fixed << std::setprecision(1) << bestConfidence << "%"
                         << "\nArea: " << (int)bestArrowArea;
                
                // Add enhanced detection details
                ArrowFeatures features = analyzeArrowFeatures(arrowCandidates[bestArrowIndex], frame);
                std::cout << "\nDetection Details:"
                         << "\n  - Corners: " << features.corner_count
                         << "\n  - Triangularity: " << std::setprecision(1) << features.triangularity_score << "%"
                         << "\n  - Tip Confidence: " << std::setprecision(1) << features.tip_confidence << "%"
                         << "\n  - Symmetry: " << std::setprecision(1) << features.symmetry_score << "%"
                         << "\n  - Edge Density: " << std::setprecision(1) << features.edge_density << "%"
                         << "\n=====================\n" << std::endl;
                
                // Draw overlay for live streaming (if overlayFrame is provided)
                if (overlayFrame != nullptr) {
                    cv::Rect bestBoundingRect = cv::boundingRect(arrowCandidates[bestArrowIndex]);
                    ArrowFeatures features = analyzeArrowFeatures(arrowCandidates[bestArrowIndex], frame);
                    
                    // Draw bounding rectangle around detected arrow
                    cv::rectangle(*overlayFrame, bestBoundingRect, cv::Scalar(0, 255, 0), 3);
                    
                    // Draw detected tip point
                    if (features.tip_confidence > 20.0) {
                        cv::circle(*overlayFrame, cv::Point((int)features.tip_point.x, (int)features.tip_point.y), 8, cv::Scalar(0, 0, 255), -1);
                        cv::putText(*overlayFrame, "TIP", 
                                   cv::Point((int)features.tip_point.x - 15, (int)features.tip_point.y - 15), 
                                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 2);
                    }
                    
                    // Draw direction text above the arrow
                    cv::putText(*overlayFrame, bestDirection, 
                               cv::Point(bestBoundingRect.x, bestBoundingRect.y - 40), 
                               cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);
                    
                    // Draw confidence percentage
                    std::string confidenceText = std::to_string((int)bestConfidence) + "%";
                    cv::putText(*overlayFrame, confidenceText, 
                               cv::Point(bestBoundingRect.x, bestBoundingRect.y - 15), 
                               cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 255), 2);
                    
                    // Draw enhanced feature info below the arrow
                    std::string featureInfo = "Corners:" + std::to_string((int)features.corner_count) + 
                                            " Tri:" + std::to_string((int)features.triangularity_score) + "%";
                    cv::putText(*overlayFrame, featureInfo, 
                               cv::Point(bestBoundingRect.x, bestBoundingRect.y + bestBoundingRect.height + 25), 
                               cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
                    
                    // Draw contour outline for better visualization
                    std::vector<std::vector<cv::Point>> contours_draw = {arrowCandidates[bestArrowIndex]};
                    cv::drawContours(*overlayFrame, contours_draw, -1, cv::Scalar(255, 0, 0), 2);
                    
                    // Add "ENHANCED ARROW DETECTED" label to distinguish this detection
                    cv::putText(*overlayFrame, "ENHANCED ARROW DETECTED", 
                               cv::Point(bestBoundingRect.x, bestBoundingRect.y - 70), 
                               cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 0), 2);
                }
                
                lastDetectedDirection = bestDirection;
                lastDetectionTime = currentTime;
            }
            
            return true;
        } else {
            // No valid arrow found
            setDetectedDirection(ARROW_NONE);
            setConfidence(0.0f);
            setTimestamp(getCurrentTimeMicros());
            return false;
        }
    }
    
    void run() {
        extern volatile bool signal_received;
        
        std::cout << "Wall-E Arrow Detection Process";
        if (streaming_enabled) {
            std::cout << " with Live Streaming";
        }
        std::cout << std::endl;
        std::cout << "==================================================" << std::endl;
        
        if (!initializeSharedMemory()) {
            std::cerr << "Failed to initialize shared memory" << std::endl;
            return;
        }
        
        std::cout << "Using rpicam-vid for video streaming" << std::endl;
        
        if (!initRpicamVideo()) {
            std::cerr << "Failed to initialize rpicam video stream" << std::endl;
            return;
        }
        
        // Start HTTP streaming server if enabled
        if (streaming_enabled) {
            if (!stream_server->start()) {
                std::cerr << "Failed to start HTTP streaming server" << std::endl;
                streaming_enabled = false;
            }
        }
        
        running = true;
        cv::Mat frame;
        int frameCount = 0;
        auto startTime = std::chrono::steady_clock::now();
        auto lastFpsTime = startTime;
        int fpsFrameCount = 0;
        
        std::cout << "Arrow detection active. Press Ctrl+C to stop." << std::endl;
        if (streaming_enabled) {
            std::cout << "Live stream available at: http://<raspberry-pi-ip>:" << streaming_port << std::endl;
        }
        
        if (standalone_mode) {
            std::cout << "Running in standalone streaming mode:\n" << std::endl;
        } else {
            std::cout << "Detection results are shared with Wall-E control system:\n" << std::endl;
        }
        
        while (running && isDetectionEnabled() && !signal_received) {
            auto loopStart = std::chrono::steady_clock::now();
            
            // Try to get a new frame (non-blocking)
            cv::Mat newFrame;
            if (cap.read(newFrame) && !newFrame.empty()) {
                frame = newFrame.clone();
                frameCount++;
                fpsFrameCount++;
            }
            
            // Process the most recent frame
            if (!frame.empty()) {
                // Create a copy for display with overlays
                cv::Mat displayFrame = frame.clone();
                
                // Process frame for arrow detection with overlay
                processFrameWithOverlay(frame, &displayFrame);
                
                // Add real-time status overlay to display frame
                std::string statusText = "Direction: " + arrowDirectionToString(getDetectedDirection());
                cv::putText(displayFrame, statusText, cv::Point(10, 30), 
                           cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);
                
                // Add system status
                std::string systemText = std::string("System: ") + (isEmergencyStop() ? "EMERGENCY" : "NORMAL") +
                                        " | Balance: " + std::to_string(getBalanceAngle()).substr(0, 5) + " deg";
                cv::putText(displayFrame, systemText, cv::Point(10, 60), 
                           cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 0), 2);
                
                // Add streaming status if enabled
                if (streaming_enabled) {
                    // Update the streaming server with the latest frame
                    stream_server->updateFrame(displayFrame);
                }
            }
            
            auto currentTime = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(currentTime - lastFpsTime);
            
            if (elapsed.count() >= 2) {
                double fps = fpsFrameCount / elapsed.count();
                std::cout << "\rProcessing at " << std::fixed << std::setprecision(1) 
                         << fps << " FPS | Frames: " << frameCount 
                         << " | Last Direction: " << arrowDirectionToString(getDetectedDirection()) 
                         << " (" << std::setprecision(1) << getConfidence() << "%)";
                if (streaming_enabled) {
                    std::cout << " | Streaming: ON";
                }
                std::cout << "     " << std::flush;
                
                lastFpsTime = currentTime;
                fpsFrameCount = 0;
            }
            
            // Update heartbeat
            setDetectionActive(true);
            
            // Check for signal again for faster response
            if (signal_received) {
                break;
            }
            
            // Ensure we process every 10ms (100 FPS max)
            auto loopEnd = std::chrono::steady_clock::now();
            auto loopDuration = std::chrono::duration_cast<std::chrono::milliseconds>(loopEnd - loopStart);
            auto remaining = std::chrono::milliseconds(10) - loopDuration;
            
            if (remaining > std::chrono::milliseconds(0)) {
                std::this_thread::sleep_for(remaining);
            }
        }
        
        cleanup();
    }
    
    void stop() {
        running = false;
        
        setDetectionActive(false);
        setDetectedDirection(ARROW_NONE);
        setConfidence(0.0f);
        
        // Stop streaming server
        if (stream_server) {
            stream_server->stop();
        }
        
        // Kill rpicam-vid processes immediately
        system("pkill -TERM rpicam-vid");
        usleep(100000); // Wait 100ms
        system("pkill -KILL rpicam-vid");
    }
    
    void cleanup() {
        std::cout << "\n🔧 Cleaning up arrow detection process..." << std::endl;
        
        setDetectionActive(false);
        setDetectedDirection(ARROW_NONE);
        setConfidence(0.0f);
        
        if (standalone_mode) {
            std::cout << "✓ Standalone mode data cleared" << std::endl;
        } else {
            std::cout << "✓ Shared memory data cleared" << std::endl;
        }
        
        if (cap.isOpened()) {
            cap.release();
            std::cout << "✓ Camera capture released" << std::endl;
        }
        
        // Stop streaming server
        if (stream_server) {
            stream_server->stop();
            std::cout << "✓ HTTP streaming server stopped" << std::endl;
        }
        
        // Kill rpicam-vid processes
        std::cout << "🔄 Stopping rpicam-vid processes..." << std::endl;
        system("pkill -TERM rpicam-vid");
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        system("pkill -KILL rpicam-vid");
        std::cout << "✓ rpicam-vid processes stopped" << std::endl;
        
        // Remove FIFO
        unlink("/tmp/rpicam_arrow_fifo");
        std::cout << "✓ FIFO pipe removed" << std::endl;
        
        std::cout << "✅ Arrow detection process cleanup complete." << std::endl;
    }
};

// Global pointer for signal handling
ArrowDetectionProcess* g_arrow_detector = nullptr;
volatile bool signal_received = false;

void signalHandler(int signum) {
    if (signal_received) {
        // Force exit if already shutting down
        std::cout << "\nForce terminating arrow detection..." << std::endl;
        exit(1);
    }
    
    signal_received = true;
    std::cout << "\n🛑 Shutdown signal received. Gracefully stopping arrow detection..." << std::endl;
    
    if (g_arrow_detector) {
        g_arrow_detector->stop();
    }
}

int main(int argc, char* argv[]) {
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    bool enable_streaming = false;
    int port = 8080;
    
    // Parse command line arguments
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--stream" || arg == "-s") {
            enable_streaming = true;
        } else if (arg == "--port" || arg == "-p") {
            if (i + 1 < argc) {
                port = std::atoi(argv[++i]);
                if (port <= 0 || port > 65535) {
                    std::cerr << "Invalid port number: " << port << std::endl;
                    return 1;
                }
            } else {
                std::cerr << "Port option requires a value" << std::endl;
                return 1;
            }
        } else if (arg == "--help" || arg == "-h") {
            std::cout << "Wall-E Arrow Detection Process\n\n";
            std::cout << "Usage: " << argv[0] << " [options]\n\n";
            std::cout << "Options:\n";
            std::cout << "  -s, --stream        Enable HTTP live streaming\n";
            std::cout << "  -p, --port PORT     Set streaming port (default: 8080)\n";
            std::cout << "  -h, --help          Show this help message\n\n";
            std::cout << "Examples:\n";
            std::cout << "  " << argv[0] << "                    # Normal operation\n";
            std::cout << "  " << argv[0] << " --stream           # With live streaming on port 8080\n";
            std::cout << "  " << argv[0] << " -s -p 8081         # With streaming on custom port\n\n";
            return 0;
        } else {
            std::cerr << "Unknown option: " << arg << std::endl;
            std::cerr << "Use --help for usage information" << std::endl;
            return 1;
        }
    }
    
    ArrowDetectionProcess detector(enable_streaming, port);
    g_arrow_detector = &detector;
    
    detector.run();
    
    return 0;
}
